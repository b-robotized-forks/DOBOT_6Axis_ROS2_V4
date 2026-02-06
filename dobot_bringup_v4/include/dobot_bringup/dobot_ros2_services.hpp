#ifndef DOBOT_BRINGUP__DOBOT_ROS2_SERVICES_HPP_
#define DOBOT_BRINGUP__DOBOT_ROS2_SERVICES_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

#include "dobot_bringup/command.h"
#include "dobot_bringup/parseTool.h"

// Service definitions
#include <dobot_msgs_v4/srv/enable_robot.hpp>
#include <dobot_msgs_v4/srv/disable_robot.hpp>
#include <dobot_msgs_v4/srv/clear_error.hpp>
#include <dobot_msgs_v4/srv/power_on.hpp>
#include <dobot_msgs_v4/srv/speed_factor.hpp>
#include <dobot_msgs_v4/srv/acc_j.hpp>
#include <dobot_msgs_v4/srv/vel_j.hpp> // SpeedJ
#include <dobot_msgs_v4/srv/set_tool.hpp>
#include <dobot_msgs_v4/srv/tool.hpp>
#include <dobot_msgs_v4/srv/get_error_id.hpp>
#include <dobot_msgs_v4/srv/robot_mode.hpp>

namespace dobot_bringup
{

class DobotRos2Services : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Dobot Ros 2 Services object
     * @param node_name Name of the ROS 2 node
     * @param commander Shared pointer to the existing CRCommanderRos2 instance
     */
    explicit DobotRos2Services(const std::string& node_name, 
                               std::shared_ptr<CRCommanderRos2> commander);

    virtual ~DobotRos2Services();

private:
    // --- Service Callbacks ---
    void enableRobotCallback(const std::shared_ptr<dobot_msgs_v4::srv::EnableRobot::Request> request,
                             std::shared_ptr<dobot_msgs_v4::srv::EnableRobot::Response> response);

    void disableRobotCallback(const std::shared_ptr<dobot_msgs_v4::srv::DisableRobot::Request> request,
                              std::shared_ptr<dobot_msgs_v4::srv::DisableRobot::Response> response);

    void clearErrorCallback(const std::shared_ptr<dobot_msgs_v4::srv::ClearError::Request> request,
                            std::shared_ptr<dobot_msgs_v4::srv::ClearError::Response> response);

    void powerOnCallback(const std::shared_ptr<dobot_msgs_v4::srv::PowerOn::Request> request,
                         std::shared_ptr<dobot_msgs_v4::srv::PowerOn::Response> response);

    void speedFactorCallback(const std::shared_ptr<dobot_msgs_v4::srv::SpeedFactor::Request> request,
                             std::shared_ptr<dobot_msgs_v4::srv::SpeedFactor::Response> response);

    void accJCallback(const std::shared_ptr<dobot_msgs_v4::srv::AccJ::Request> request,
                      std::shared_ptr<dobot_msgs_v4::srv::AccJ::Response> response);

    void speedJCallback(const std::shared_ptr<dobot_msgs_v4::srv::VelJ::Request> request,
                        std::shared_ptr<dobot_msgs_v4::srv::VelJ::Response> response);

    void setToolCallback(const std::shared_ptr<dobot_msgs_v4::srv::SetTool::Request> request,
                         std::shared_ptr<dobot_msgs_v4::srv::SetTool::Response> response);

    void toolCallback(const std::shared_ptr<dobot_msgs_v4::srv::Tool::Request> request,
                      std::shared_ptr<dobot_msgs_v4::srv::Tool::Response> response);

    void getErrorIDCallback(const std::shared_ptr<dobot_msgs_v4::srv::GetErrorID::Request> request,
                            std::shared_ptr<dobot_msgs_v4::srv::GetErrorID::Response> response);

    void robotModeCallback(const std::shared_ptr<dobot_msgs_v4::srv::RobotMode::Request> request,
                           std::shared_ptr<dobot_msgs_v4::srv::RobotMode::Response> response);
    
    // Shared instance of commander passed from HW interface
    std::shared_ptr<CRCommanderRos2> commander_;
    
    // Callback group to enforce serialization of service requests within this node
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // Service Servers
    rclcpp::Service<dobot_msgs_v4::srv::EnableRobot>::SharedPtr srv_enable_robot_;
    rclcpp::Service<dobot_msgs_v4::srv::DisableRobot>::SharedPtr srv_disable_robot_;
    rclcpp::Service<dobot_msgs_v4::srv::ClearError>::SharedPtr srv_clear_error_;
    rclcpp::Service<dobot_msgs_v4::srv::PowerOn>::SharedPtr srv_power_on_;
    rclcpp::Service<dobot_msgs_v4::srv::SpeedFactor>::SharedPtr srv_speed_factor_;
    rclcpp::Service<dobot_msgs_v4::srv::AccJ>::SharedPtr srv_acc_j_;
    rclcpp::Service<dobot_msgs_v4::srv::VelJ>::SharedPtr srv_speed_j_;
    rclcpp::Service<dobot_msgs_v4::srv::SetTool>::SharedPtr srv_set_tool_;
    rclcpp::Service<dobot_msgs_v4::srv::Tool>::SharedPtr srv_tool_;
    rclcpp::Service<dobot_msgs_v4::srv::GetErrorID>::SharedPtr srv_get_error_id_;
    rclcpp::Service<dobot_msgs_v4::srv::RobotMode>::SharedPtr srv_robot_mode_;
};

} // namespace dobot_bringup

#endif // DOBOT_BRINGUP__DOBOT_ROS2_SERVICES_HPP_