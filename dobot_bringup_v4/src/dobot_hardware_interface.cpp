#include "dobot_bringup/dobot_hardware_interface.hpp"

#include <limits>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

namespace dobot_hardware_interface
{

inline void bit_set(uint16_t &state, int index) {
    state |= (1 << index);
}

inline void bit_clear(uint16_t &state, int index) {
    state &= ~(1 << index);
}

inline bool bit_get(uint16_t state, int index) {
    return (state >> index) & 1;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{   
    // Fixed to 125Hz, as per documentation, the RT robot state feedback comes every 8ms = 125Hz
    // https://docs.trossenrobotics.com/dobot_cr_cobots_docs/tcpip_protocol/functions.html#message-format
    if (info_.rw_rate != 125)
    {
        RCLCPP_FATAL(getLogger(), "Hardware interface loop rate is %u Hz, but 125 Hz is required!", info_.rw_rate);
        return hardware_interface::CallbackReturn::ERROR;
    }

    std::string robot_ip;

    if (info_.hardware_parameters.count("robot_ip"))
    {
        robot_ip = info_.hardware_parameters.at("robot_ip");
    }
    else
    {
        RCLCPP_ERROR(getLogger(), "No 'robot_ip' parameter provided! Check your ros2_control.urdf.xacro file.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters.count("gpio_rw_rate")) {
        try {
            gpio_rw_rate_ = std::stod(info_.hardware_parameters.at("gpio_rw_rate"));
        } catch(const std::invalid_argument& e) {
            RCLCPP_WARN(getLogger(), "Invalid format for gpio_rw_rate, using default 10.0Hz");
        }
    }
    
    if (gpio_rw_rate_ > 20.0) {
        RCLCPP_WARN(getLogger(), "gpio_rw_rate %f Hz exceeds max 20Hz. Clamping to 20Hz.", gpio_rw_rate_);
        gpio_rw_rate_ = 20.0;
    }
    if (gpio_rw_rate_ <= 0.0) {
        RCLCPP_WARN(getLogger(), "gpio_rw_rate must be positive. Using default 10.0Hz.");
        gpio_rw_rate_ = 10.0;
    }

    try 
    {
        commander_ = std::make_shared<CRCommanderRos2>(robot_ip);
        commander_->init(); // Starts the recvTask thread
        
        // Wait for connection
        int retries = 0;
        while (!commander_->isConnected() && retries < 20) {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
            retries++;
        }

        if (!commander_->isConnected()) {
            RCLCPP_WARN(getLogger(), "Dobot not yet connected after %d retries.", retries);
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(getLogger(), "Dobot connected successfully.");
        }

    } 
    catch (const std::exception &e) 
    {
        RCLCPP_ERROR(getLogger(), "Exception during configuration: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Activating Dobot Hardware Interface...");
    
    int32_t err_id = 0;

    // Enable Robot
    RCLCPP_INFO(getLogger(), "Sending EnableRobot() command...");
    if (!commander_->callRosService("EnableRobot()", err_id)) {
        RCLCPP_ERROR(getLogger(), "Failed to send EnableRobot request.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (err_id != 0) {
        RCLCPP_WARN(getLogger(), "EnableRobot returned error ID: %d", err_id);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Wait a bit for the robot to enable
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    
    auto data = *commander_->getRealData();

    int retries = 0;
    while(retries < 10)
    {
        data = *commander_->getRealData();
        if(data.len > 0) break;
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        retries++;
    }

    if(retries >= 10)
    {
        RCLCPP_ERROR(getLogger(), "Could not get valid RealTimeData for Activation.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Check for errors
    if (data.robot_mode == 9)
    {
        RCLCPP_WARN(getLogger(), "Robot is in ERROR state. Sending ClearError()...");
        if (!commander_->callRosService("ClearError()", err_id)) {
            RCLCPP_ERROR(getLogger(), "Failed to send ClearError() request.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (err_id != 0) {
            RCLCPP_WARN(getLogger(), "EnableRobot returned error ID: %d", err_id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        if(commander_->getRealData()->robot_mode == 9){
            RCLCPP_ERROR(getLogger(), "Robot is still in ERROR state. Aborting.");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Enable again
        RCLCPP_INFO(getLogger(), "Sending EnableRobot() command...");
        if (!commander_->callRosService("EnableRobot()", err_id)) {
            RCLCPP_ERROR(getLogger(), "Failed to send EnableRobot request.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (err_id != 0) {
            RCLCPP_WARN(getLogger(), "EnableRobot returned error ID: %d", err_id);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    if (!commander_->isEnable()) {
        RCLCPP_ERROR(getLogger(), "Robot is not Enabled, when expected to be. Robot Mode: %lu", (unsigned long)data.robot_mode);
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(getLogger(), "Robot is enabled. Setting state to command...");

    // Sync initial commands with current state to avoid jumps
    set_command("joint1/position", data.q_actual[0] * DEG_TO_RAD   );
    set_command("joint2/position", data.q_actual[1] * DEG_TO_RAD   );
    set_command("joint3/position", data.q_actual[2] * DEG_TO_RAD   );
    set_command("joint4/position", data.q_actual[3] * DEG_TO_RAD   );
    set_command("joint5/position", data.q_actual[4] * DEG_TO_RAD   );
    set_command("joint6/position", data.q_actual[5] * DEG_TO_RAD   );

    // Sync initial GPIO state
    // digital_outputs is uint64_t where bit 0 is DO1.

    gpio_command_rt_ = static_cast<uint16_t>(data.digital_outputs & 0xFFFF);
    for (int i = 0; i < 16; ++i)
    {
        double val = (gpio_command_rt_ & (1 << i)) ? 1.0 : 0.0;
        set_command("DO/" + std::to_string(i + 1), val);
    }
    gpio_command_nrt_.store(gpio_command_rt_, std::memory_order_relaxed);

    // Start GPIO thread
    gpio_nrt_thread_running_ = true;
    gpio_nrt_thread_ = std::thread(&DobotHardwareInterface::gpio_nrt_thread_func, this);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Deactivating Dobot Hardware Interface...");
    
    gpio_nrt_thread_running_ = false;
    if (gpio_nrt_thread_.joinable()) {
        gpio_nrt_thread_.join();
    }

    if (commander_) {
        int32_t err_id = 0;
        RCLCPP_INFO(getLogger(), "Sending Stop() command...");
        commander_->callRosService("Stop()", err_id);
    }
    
    // We do NOT reset the commander here, because we want to maintain the connection
    // in case of re-activation. The connection is managed in configure/shutdown.
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    gpio_nrt_thread_running_ = false;
    if (gpio_nrt_thread_.joinable()) {
        gpio_nrt_thread_.join();
    }

    commander_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DobotHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // if disconnected, unconfigure
    if (!commander_ || !commander_->isConnected() ) return hardware_interface::return_type::ERROR;

    populate_state_interfaces(*commander_->getRealData());

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{ 
    // if disconnected, unconfigure
    if (!commander_ || !commander_->isConnected() ) return hardware_interface::return_type::ERROR; 
    // if disabled, deactivate
    if (!commander_->isEnable()) return hardware_interface::return_type::DEACTIVATE;

    write_command_ServoJ();
    write_command_DOGroup();

    return hardware_interface::return_type::OK;
}

void DobotHardwareInterface::populate_state_interfaces(const RealTimeData& data)
{
    // for now, we hardcode interfaces we want for position control, similar to UR
    // https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/83aeea4c836849d0c5c0557810ec25e8b33ae9bd/ur_robot_driver/src/hardware_interface.cpp#L251-L262

    // later we add more and get smarter about populating this data.
    set_state("joint1/position", data.q_actual[0] * DEG_TO_RAD   );
    set_state("joint2/position", data.q_actual[1] * DEG_TO_RAD   );
    set_state("joint3/position", data.q_actual[2] * DEG_TO_RAD   );
    set_state("joint4/position", data.q_actual[3] * DEG_TO_RAD   );
    set_state("joint5/position", data.q_actual[4] * DEG_TO_RAD   );
    set_state("joint6/position", data.q_actual[5] * DEG_TO_RAD   );

    set_state("joint1/velocity", data.qd_actual[0] * DEG_TO_RAD  );
    set_state("joint2/velocity", data.qd_actual[1] * DEG_TO_RAD  );
    set_state("joint3/velocity", data.qd_actual[2] * DEG_TO_RAD  );
    set_state("joint4/velocity", data.qd_actual[3] * DEG_TO_RAD  );
    set_state("joint5/velocity", data.qd_actual[4] * DEG_TO_RAD  );
    set_state("joint6/velocity", data.qd_actual[5] * DEG_TO_RAD  );

    // Populate DI state
    uint64_t di_bits = data.digital_input_bits;
    for (int i = 0; i < 32; ++i)
    {
        double val = (di_bits & (1ULL << i)) ? 1.0 : 0.0;
        set_state("DI/" + std::to_string(i + 1), val);
    }

    // Populate DO state feedback
    uint64_t do_bits = data.digital_outputs;
    for (int i = 0; i < 16; ++i)
    {
        double val = (do_bits & (1ULL << i)) ? 1.0 : 0.0;
        set_state("DO/" + std::to_string(i + 1), val);
    }

    return;
}

void DobotHardwareInterface::write_command_ServoJ(){

    std::array<double, 6> joint_commands;
    joint_commands[0] = get_command("joint1/position");
    joint_commands[1] = get_command("joint2/position");
    joint_commands[2] = get_command("joint3/position");
    joint_commands[3] = get_command("joint4/position");
    joint_commands[4] = get_command("joint5/position");
    joint_commands[5] = get_command("joint6/position");

    for (size_t i = 0; i < 6; ++i)
    {
        // Only continue if all joint positions hold a valid command
        if (std::isnan(joint_commands[i])) return;
    }

    char cmd_string[128];
        
    std::snprintf(cmd_string, sizeof(cmd_string), 
        "ServoJ(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)",
        joint_commands[0] * RAD_TO_DEG,
        joint_commands[1] * RAD_TO_DEG,
        joint_commands[2] * RAD_TO_DEG,
        joint_commands[3] * RAD_TO_DEG,
        joint_commands[4] * RAD_TO_DEG,
        joint_commands[5] * RAD_TO_DEG
    );

    commander_->tcpSendServoJ(std::string(cmd_string));

    return;
}

void DobotHardwareInterface::write_command_DOGroup()
{  
    // preserve current command
    uint16_t next_state = gpio_command_rt_;

    // check if any bits should be updated
    for (int i = 0; i < 16; ++i)
    {
        double val = get_command("DO/" + std::to_string(i + 1));

        // If command is valid (not NaN), apply it to our state
        if (!std::isnan(val))
        {
            if (val > 0.5) {
                bit_set(next_state, i);
            } else {
                bit_clear(next_state, i);
            }
        }
        // If NaN, next_state retains bit 'i' from gpio_command_rt_
    }

    // Only write to nrt member if the desired command changed
    if (next_state != gpio_command_rt_)
    {
        gpio_command_rt_ = next_state;
        gpio_command_nrt_.store(next_state, std::memory_order_relaxed);
    }
}

void DobotHardwareInterface::gpio_nrt_thread_func()
{
    // Init thread local history of commands sent
    uint16_t command_last_sent = gpio_command_nrt_.load(std::memory_order_relaxed);

    rclcpp::Rate rate(gpio_rw_rate_);

    while(gpio_nrt_thread_running_)
    {
        uint16_t command_desired = gpio_command_nrt_.load(std::memory_order_relaxed);

        // XOR: which bits changed?
        uint16_t diff = command_desired ^ command_last_sent;
        
        if (diff != 0)
        {
            std::stringstream ss;
            ss << "DOGroup(";
            
            for (int i = 0; i < 16; ++i)
            {   
                // if bit changed, add to command
                if (bit_get(diff, i))
                {
                    int val = bit_get(command_desired, i);
                    ss << (i + 1) << "," << val << ",";
                }
            }

            // pop the trailing comma
            std::string cmd_str = ss.str();
            if (cmd_str.back() == ',') {
                cmd_str.pop_back();
            }
            
            cmd_str += ")";
            
            int32_t err_id = 0;
            // blocking call! atleast 10ms
            if (!commander_->callRosService(cmd_str, err_id))
            {
                RCLCPP_WARN(getLogger(), "Failed to send GPIO command, retrying next cycle...");
                
            }
            else if (err_id != 0) {
                RCLCPP_WARN(getLogger(), "DOGroup() returned error ID: %d. Retrying next cycle...", err_id);
            }
            else
            {
                command_last_sent = command_desired;
            }
            
        }

        rate.sleep();
    }
}

} // namespace dobot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dobot_hardware_interface::DobotHardwareInterface, hardware_interface::SystemInterface)
