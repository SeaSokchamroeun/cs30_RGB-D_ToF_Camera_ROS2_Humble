# cs30_RGB-D_ToF_Camera_ROS2_Humble
This is a repository is for cs30 camera running with Ubuntu 22.04 with ROS2 Humble

### Recommended OS: Ubuntu 22.04 (Strongly Advised)
### Recommended ROS: ROS2
```
cs30_ws/
└── workspace/
    ├── build/
    ├── install/
    ├── log/
    └── src/
        └── synexens_ros2/
            ├── camera_config/
            │   └── default.yaml
            ├── ext/
            │   └── sdk/                 # <--- SDK contents collapsed
            ├── include/
            │   └── synexens_ros2/
            │       ├── SYCalibrationTransformData.h
            │       ├── SYRosDevice.h
            │       ├── SYRosDeviceParmas.h
            │       └── SYRosTypes.h
            ├── launch/
            │   ├── convert_rgb.launch.py
            │   ├── driver_launch.py
            │   ├── view_camera1.launch.py
            │   └── viewer_launch.py
            ├── rviz/
            │   └── view.rviz
            ├── script/
            │   ├── opencv_rgbIR_viewer.py
            │   ├── opencv_rgb_viewer.py
            │   ├── setup.sh
            │   └── synexens-usb.rules
            ├── CMakeLists.txt
            └── package.xml
```

1. Install ROS 2 dependencies (if not already installed)
```bash
sudo apt update
sudo apt install ros-humble-camera-info-manager \
                 ros-humble-image-proc \
                 ros-humble-rqt-image-view \
                 ros-humble-image-view \
                 python3-pip
```

(ROS2 HUMBLE requires Numpy<2)
```bash
pip3 install "numpy<2" --user
```

2. Clone & Build Workspace
```bash
# Create workspace
mkdir -p ~/cs30_ws/src
cd ~/cs30_ws/src

# Clone your repository (replace with your actual URL)
git clone https://github.com/SeaSokchamroeun/synexens_ros2.git

# Install udev rules (for USB permissions)
cd synexens_ros2/script
sudo ./setup.sh

# Return to workspace root
cd ~/cs30_ws

# Build
colcon build --packages-select synexens_ros2

# Source the workspace
source install/setup.bash
```

3. How to launch the camera with sdk
```bash
cd ~/cs30_ws
source install/setup.bash
ros2 launch synexens_ros2 driver_launch.py
```
In another Terminal, Use opencv script to view the camera
```bash
ros2 run synexens_ros2 opencv_rgb_viewer_only.py
```

Run this to see the existing ROS2 topic
```bash
ros2 topic list
```

Hwat you should expect:
```
/camera1_HV0130315L0317/rgb_raw
/camera1_HV0130315L0317/ir_raw
/camera1_HV0130315L0317/depth_raw
/camera1_HV0130315L0317/points2
/camera1_HV0130315L0317/rgb_info
/camera1_HV0130315L0317/ir_info
/camera1_HV0130315L0317/depth_info
/parameter_events
/rosout
/tf_static
```
