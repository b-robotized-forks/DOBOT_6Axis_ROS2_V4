#include <limits>
#include <vector>
#include <cstdint>
#include <regex>
#include <charconv> //std::from_chars

#include "dobot_hardware_interface/dobot_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dobot_hardware_interface
{
hardware_interface::CallbackReturn DobotHardwareInterface::on_init(
    const hardware_interface::HardwareComponentParams & /*params*/)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DobotHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
{ 
    return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn DobotHardwareInterface::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}


} // namespace dobot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    dobot_hardware_interface::DobotHardwareInterface, hardware_interface::SystemInterface)