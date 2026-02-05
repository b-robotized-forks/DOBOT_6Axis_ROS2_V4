#include "dobot_bringup/dobot_hardware_interface_gpio.hpp"

#include <limits>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dobot_hardware_interface_gpio
{

hardware_interface::CallbackReturn DobotHardwareInterfaceGPIO::on_configure(
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

    if (info_.rw_rate != 5)
    {
        RCLCPP_WARN(getLogger(), "Hardware interface loop rate is %u Hz, but 5 Hz is recommended for GPIO!", info_.rw_rate);
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
            RCLCPP_WARN(getLogger(), "Dobot GPIO not yet connected after %d retries.", retries);
            return hardware_interface::CallbackReturn::ERROR;
        } else {
            RCLCPP_INFO(getLogger(), "Dobot GPIO connected successfully.");
        }

        // Initialize previous_commands_ with size 16 (for 16 DOs) and default value
        // We will sync with actual state in on_activate
        previous_commands_.resize(16, std::numeric_limits<double>::quiet_NaN());

    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(getLogger(), "Exception during configuration: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterfaceGPIO::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(getLogger(), "Activating Dobot GPIO Hardware Interface...");

    // Wait for valid data
    auto data = commander_->getRealData();
    int retries = 0;
    while(retries < 10)
    {
        data = commander_->getRealData();
        if(data->len > 0) break;
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        retries++;
    }

    if(retries >= 10)
    {
        RCLCPP_ERROR(getLogger(), "Could not get valid RealTimeData for GPIO sync.");
        hardware_interface::CallbackReturn::ERROR;
    }

    // Sync initial commands with current state
    uint64_t do_bits = data.digital_outputs;
    
    for (int i = 0; i < 16; ++i)
    {
        double val = (do_bits & (1ULL << i)) ? 1.0 : 0.0;
        // Set the command interface to the current value
        set_command("DO" + std::to_string(i + 1), val);
        previous_commands_[i] = val;
    }
    RCLCPP_INFO(getLogger(), "GPIO Commands synced with robot state.");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterfaceGPIO::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DobotHardwareInterfaceGPIO::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!commander_ || !commander_->isConnected()) return hardware_interface::return_type::ERROR;

    auto data = *commander_->getRealData();
    uint64_t di_bits = data.digital_input_bits;
    
    // We expect interfaces named "1" through "32" for DI GPIO
    for (int i = 0; i < 32; ++i)
    {        
        double val = (di_bits & (1ULL << i)) ? 1.0 : 0.0;
        
        set_state("DI/" + std::to_string(i + 1), val);
    }

    // We expect interfaces named "1" through "16" for DO GPIO
    uint64_t do_bits = data.digital_outputs;

    for (int i = 0; i < 16; ++i)
    {
        std::string interface_name = std::to_string(i + 1);
        double val = (do_bits & (1ULL << i)) ? 1.0 : 0.0;
        set_state("DO/" + interface_name, val);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotHardwareInterfaceGPIO::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (!commander_ || !commander_->isConnected()) return hardware_interface::return_type::ERROR;

    write_command_DOGroup();

    return hardware_interface::return_type::OK;
}

void DobotHardwareInterfaceGPIO::write_command_DOGroup()
{
    // If any IO changed since last write, send a command.
    bool any_change = false;
    std::stringstream ss;
    ss << "DOGroup(";
    
    std::array<int, 16> changed_indices;
    std::array<int, 16> new_values;
    size_t change_count = 0;

    for (int i = 0; i < 16; ++i)
    {
        double cmd_val = get_command("DO/" + std::to_string(i + 1););
        
        if (std::isnan(cmd_val)) continue;

        // anything > 0.5 is 1, else 0
        double current_binary = (cmd_val > 0.5) ? 1.0 : 0.0;
        double prev_binary = (previous_commands_[i] > 0.5) ? 1.0 : 0.0;

        if (std::abs(current_binary - prev_binary) > 0.1 || std::isnan(previous_commands_[i]))
        {
            any_change = true;
            changed_indices[change_count] = i + 1; // 1-based index for robot command
            new_values[change_count] = (int)current_binary;
            change_count++;
            previous_commands_[i] = current_binary;
        }
    }

    if (any_change)
    {
        // command string: DOGroup(index1, val1, index2, val2, ...)
        for (size_t k = 0; k < change_count; ++k)
        {
            if (k > 0) ss << ",";
            ss << changed_indices[k] << "," << new_values[k];
        }
        ss << ")";
        
        std::string cmd_str = ss.str();
        RCLCPP_INFO(getLogger(), "Sending GPIO Command: %s", cmd_str.c_str());
        
        int32_t err_id = 0;
        commander_->callRosService(cmd_str, err_id);
    }
}

} // namespace dobot_hardware_interface_gpio

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dobot_hardware_interface_gpio::DobotHardwareInterfaceGPIO, hardware_interface::SystemInterface)
