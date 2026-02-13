#ifndef DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_HPP_
#define DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <array>
#include <cmath>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "dobot_bringup/command.h"
#include "dobot_bringup/dobot_ros2_services.hpp"

namespace dobot_hardware_interface
{
    
class DobotHardwareInterface : public hardware_interface::SystemInterface
{
public:
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
    rclcpp::Logger getLogger() { return rclcpp::get_logger(info_.name); }
    
    void populate_state_interfaces(const RealTimeData& data);
    void write_command_ServoJ();
    // ServoJ command parameters
    double servoJ_t_ = 0.1;
    double servoJ_lookahead_ = 50;
    double servoJ_gain_ = 300;
    // Prefix from urdf, so we can correctly namespace joint/gpios in multi-robot scenarios
    std::string prefix_ = ""; 

    void write_command_DOGroup();
    void gpio_nrt_thread_func();

    std::shared_ptr<CRCommanderRos2> commander_;

    // GPIO Background Thread
    double gpio_rw_rate_ = 10.0; // 10Hz default
    std::thread gpio_nrt_thread_;
    std::atomic<bool> gpio_nrt_thread_running_;
    
    uint16_t gpio_command_rt_ = 0; // for parsing the current command_interfaces for gpios
    std::atomic<uint16_t> gpio_command_nrt_{0}; // what the gpio thread finally sees

    // ROS2 services
    std::shared_ptr<dobot_bringup::DobotRos2Services> service_node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> service_executor_;
    std::thread service_thread_;
};

} // namespace dobot_hardware_interface

#endif // DOBOT_HARDWARE_INTERFACE__DOBOT_SYSTEM_HPP_
