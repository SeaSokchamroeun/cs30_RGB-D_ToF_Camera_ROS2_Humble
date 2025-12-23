#include "synexens_ros2/SYRosDeviceParmas.h"

void SYRosDeviceParmas::SetParams(rclcpp::Node::SharedPtr node_)
{
    // Prefix added to tf frame IDs. It typically contains a trailing '_' unless empty.
    node_->declare_parameter<std::string>("tf_prefix", "");
    node_->get_parameter("tf_prefix", config_params_.tf_prefix);
    // The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30
    node_->declare_parameter<int>("fps", 30);
    node_->get_parameter("fps", config_params_.fps);
    // Generate a point cloud from depth data. Requires depth_enabled
    node_->declare_parameter<bool>("point_cloud_enabled", true);
    node_->get_parameter("point_cloud_enabled", config_params_.point_cloud_enabled);
    // Mapping 1 Depth to RGB 0 RGB to Depth default 0
    node_->declare_parameter<int>("mapping_mode", 0);
    node_->get_parameter("mapping_mode", config_params_.mapping_mode);
}

SYCameraParams SYRosDeviceParmas::GetParams()
{
    return config_params_;
}