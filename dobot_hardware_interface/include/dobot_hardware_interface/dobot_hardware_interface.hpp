#ifndef DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_HPP_
#define DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_HPP_

#include <string>
#include <vector>
#include <limits>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dobot_hardware_interface
{
    
class DobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentParams &params);

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    rclcpp::Logger getLogger() { return rclcpp::get_logger("DobotHardwareInterface"); }
};

} // namespace dobot_hardware_interface

#endif // DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_HPP_