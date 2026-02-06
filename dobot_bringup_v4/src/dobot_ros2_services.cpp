#include "dobot_bringup/dobot_ros2_services.hpp"

namespace dobot_bringup
{

DobotRos2Services::DobotRos2Services(const std::string& node_name, 
                                     std::shared_ptr<CRCommanderRos2> commander)
    : Node(node_name), commander_(commander)
{
    if (!commander_) {
        RCLCPP_ERROR(this->get_logger(), "DobotRos2Services received null commander!");
    }
    
    // Create a Mutually Exclusive callback group to serialize service calls within this node
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    srv_enable_robot_ = create_service<dobot_msgs_v4::srv::EnableRobot>(
        "EnableRobot",
        std::bind(&DobotRos2Services::enableRobotCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_disable_robot_ = create_service<dobot_msgs_v4::srv::DisableRobot>(
        "DisableRobot",
        std::bind(&DobotRos2Services::disableRobotCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_clear_error_ = create_service<dobot_msgs_v4::srv::ClearError>(
        "ClearError",
        std::bind(&DobotRos2Services::clearErrorCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_power_on_ = create_service<dobot_msgs_v4::srv::PowerOn>(
        "PowerOn",
        std::bind(&DobotRos2Services::powerOnCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_speed_factor_ = create_service<dobot_msgs_v4::srv::SpeedFactor>(
        "SpeedFactor",
        std::bind(&DobotRos2Services::speedFactorCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_acc_j_ = create_service<dobot_msgs_v4::srv::AccJ>(
        "AccJ",
        std::bind(&DobotRos2Services::accJCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_speed_j_ = create_service<dobot_msgs_v4::srv::VelJ>(
        "SpeedJ",
        std::bind(&DobotRos2Services::speedJCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_set_tool_ = create_service<dobot_msgs_v4::srv::SetTool>(
        "SetTool",
        std::bind(&DobotRos2Services::setToolCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_tool_ = create_service<dobot_msgs_v4::srv::Tool>(
        "Tool",
        std::bind(&DobotRos2Services::toolCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_get_error_id_ = create_service<dobot_msgs_v4::srv::GetErrorID>(
        "GetErrorID",
        std::bind(&DobotRos2Services::getErrorIDCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    srv_robot_mode_ = create_service<dobot_msgs_v4::srv::RobotMode>(
        "RobotMode",
        std::bind(&DobotRos2Services::robotModeCallback, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        callback_group_);

    RCLCPP_INFO(this->get_logger(), "DobotRos2Services initialized.");
}

DobotRos2Services::~DobotRos2Services()
{
}

void DobotRos2Services::enableRobotCallback(const std::shared_ptr<dobot_msgs_v4::srv::EnableRobot::Request> request,
                                            std::shared_ptr<dobot_msgs_v4::srv::EnableRobot::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parserenableRobotRequest2String(request), response->res)) {
        response->res = -1; // Indicate error/busy
    }
}

void DobotRos2Services::disableRobotCallback(const std::shared_ptr<dobot_msgs_v4::srv::DisableRobot::Request> request,
                                             std::shared_ptr<dobot_msgs_v4::srv::DisableRobot::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parserdisableRobotRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::clearErrorCallback(const std::shared_ptr<dobot_msgs_v4::srv::ClearError::Request> request,
                                           std::shared_ptr<dobot_msgs_v4::srv::ClearError::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parserclearErrorRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::powerOnCallback(const std::shared_ptr<dobot_msgs_v4::srv::PowerOn::Request> request,
                                        std::shared_ptr<dobot_msgs_v4::srv::PowerOn::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parserpowerOnRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::speedFactorCallback(const std::shared_ptr<dobot_msgs_v4::srv::SpeedFactor::Request> request,
                                            std::shared_ptr<dobot_msgs_v4::srv::SpeedFactor::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parserspeedFactorRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::accJCallback(const std::shared_ptr<dobot_msgs_v4::srv::AccJ::Request> request,
                                     std::shared_ptr<dobot_msgs_v4::srv::AccJ::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parseraccJRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::speedJCallback(const std::shared_ptr<dobot_msgs_v4::srv::VelJ::Request> request,
                                       std::shared_ptr<dobot_msgs_v4::srv::VelJ::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parservelJRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::setToolCallback(const std::shared_ptr<dobot_msgs_v4::srv::SetTool::Request> request,
                                        std::shared_ptr<dobot_msgs_v4::srv::SetTool::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parserSetToolRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::toolCallback(const std::shared_ptr<dobot_msgs_v4::srv::Tool::Request> request,
                                     std::shared_ptr<dobot_msgs_v4::srv::Tool::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService(parseTool::parsertoolRequest2String(request), response->res)) {
        response->res = -1;
    }
}

void DobotRos2Services::getErrorIDCallback(const std::shared_ptr<dobot_msgs_v4::srv::GetErrorID::Request> request,
                                           std::shared_ptr<dobot_msgs_v4::srv::GetErrorID::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService_f(parseTool::parserGetErrorIDRequest2String(request), response->res, response->robot_return)) {
        response->res = -1;
    }
}

void DobotRos2Services::robotModeCallback(const std::shared_ptr<dobot_msgs_v4::srv::RobotMode::Request> request,
                                          std::shared_ptr<dobot_msgs_v4::srv::RobotMode::Response> response)
{
    if (!commander_) return;
    if (!commander_->callRosService_f(parseTool::parserrobotModeRequest2String(request), response->res, response->robot_return)) {
        response->res = -1;
    }
}

} // namespace dobot_bringup