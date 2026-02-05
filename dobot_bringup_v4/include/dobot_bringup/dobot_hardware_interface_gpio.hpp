#ifndef DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_GPIO_HPP_
#define DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_GPIO_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <array>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dobot_bringup/command.h"

namespace dobot_hardware_interface_gpio
{
    
class DobotHardwareInterfaceGPIO : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    rclcpp::Logger getLogger() { return rclcpp::get_logger("DobotHardwareInterfaceGPIO"); }
    
    std::shared_ptr<CRCommanderRos2> commander_;
    
    // only send commands when values change
    std::array<double, 16> previous_commands_;

    void write_command_DOGroup();
};

} // namespace dobot_hardware_interface_gpio

#endif // DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_GPIO_HPP_
