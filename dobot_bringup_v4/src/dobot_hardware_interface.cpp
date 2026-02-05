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

hardware_interface::CallbackReturn DobotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{   
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

    // Fixed to 125Hz, as per documentation, the RT robot state feedback comes every 8ms = 125Hz
    // https://docs.trossenrobotics.com/dobot_cr_cobots_docs/tcpip_protocol/functions.html#message-format
    if (info_.rw_rate != 125)
    {
        RCLCPP_FATAL(getLogger(), "Hardware interface loop rate is %u Hz, but 125 Hz is required!", info_.rw_rate);
        return hardware_interface::CallbackReturn::ERROR;
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
    
    // ROBOT MODES:
    // https://docs.trossenrobotics.com/dobot_cr_cobots_docs/tcpip_protocol/functions.html#robotmode

    // Error code descriptions:
    // https://docs.trossenrobotics.com/dobot_cr_cobots_docs/tcpip_protocol/functions.html#error-code-descriptions
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

    // Check for errors
    if (commander_->isError())
    {
        RCLCPP_WARNING(getLogger(), "Robot is in ERROR state. Sending ClearError()...");
        if (!commander_->callRosService("ClearError()", err_id)) {
            RCLCPP_ERROR(getLogger(), "Failed to send ClearError() request.");
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (err_id != 0) {
            RCLCPP_WARN(getLogger(), "EnableRobot returned error ID: %d", err_id);
            return hardware_interface::CallbackReturn::ERROR;
        }

        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        if(commander_->isError()){
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

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Deactivating Dobot Hardware Interface...");
    commander_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
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

} // namespace dobot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dobot_hardware_interface::DobotHardwareInterface, hardware_interface::SystemInterface)
