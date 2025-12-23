#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "synexens_ros2/SYRosDevice.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr synexens_ros2_node = rclcpp::Node::make_shared("synexens_ros2_node", options);

  std::shared_ptr<SYRosDevice> device = std::make_shared<SYRosDevice>(synexens_ros2_node);
  
  SYErrorCode errorCode = device->startCameras();

  if (errorCode != Synexens::SYERRORCODE_SUCCESS)
  {
    RCLCPP_ERROR_STREAM(synexens_ros2_node->get_logger(), "Failed to start cameras");
    return -1;
  }

  RCLCPP_INFO(synexens_ros2_node->get_logger(), "ROS Exit Started");
  device.reset();
  RCLCPP_INFO(synexens_ros2_node->get_logger(), "ROS Exit");
  rclcpp::shutdown();
  RCLCPP_INFO(synexens_ros2_node->get_logger(), "ROS Shutdown complete");
  return 0;
}
