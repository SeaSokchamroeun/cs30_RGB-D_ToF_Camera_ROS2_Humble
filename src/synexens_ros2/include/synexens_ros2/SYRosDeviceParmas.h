#ifndef SYRosDeviceParmas_H
#define SYRosDeviceParmas_H

#include "SYRosTypes.h"
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std;

// config params struct
struct SYCameraParams
{
    string tf_prefix = "";
    int fps = 30;
    bool point_cloud_enabled = true;
    int mapping_mode = 1;
};

class SYRosDeviceParmas
{
public:
    // set params
    void SetParams(rclcpp::Node::SharedPtr node_);
    // get parmas
    SYCameraParams GetParams(); 
private:
    SYCameraParams config_params_;
};

#endif