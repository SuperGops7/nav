#include "rclcpp/rclcpp.hpp"
#include "turtlebot_interfaces/srv/trajectory_request.hpp"

#include <memory>

void get_trajectory(const std::shared_ptr<turtlebot_interfaces::srv::TrajectoryRequest::Request> request,
          std::shared_ptr<turtlebot_interfaces::srv::TrajectoryRequest::Response>response)
{
  response->success = true;
  response->message = "Trajectory captured successfully!";
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %s" " b: %f",
                request->filename.c_str(), request->duration);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("trajectory_capture_server");

  rclcpp::Service<turtlebot_interfaces::srv::TrajectoryRequest>::SharedPtr service =
    node->create_service<turtlebot_interfaces::srv::TrajectoryRequest>("trajectory_capture_service", &get_trajectory);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to capture trajectory.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}