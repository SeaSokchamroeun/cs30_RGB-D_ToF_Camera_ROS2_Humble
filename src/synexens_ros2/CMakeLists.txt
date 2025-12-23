cmake_minimum_required(VERSION 3.8)
project(synexens_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(camera_info_manager REQUIRED)

############################################
############ Synexens SDK start ############
############################################
set(SY_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk)
set(SY_INCLUDE_DIR ${SY_SDK_PATH}/include)
set(SY_SDK_LIBS_DIR ${SY_SDK_PATH}/lib)
set(SY_SDK_LIBS SynexensSDK csreconstruction2.0 SonixCamera ${OpenCV_LIBS})

set(OPENCV_LIB_DIR ${SY_SDK_PATH}/opencv)

link_directories(${SY_SDK_LIBS_DIR} ${OPENCV_LIB_DIR})
##########################################
############ Synexens SDK END ############
##########################################

###### Synexens Node Start ######
add_executable(${PROJECT_NAME}_node src/SYRosNode.cpp src/SYRosDevice.cpp src/SYCalibrationTransformData.cpp src/SYRosDeviceParams.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ext/sdk/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  "geometry_msgs"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "angles"
  "camera_info_manager"
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${SY_SDK_LIBS}
)
###### Synexens Node End ######


###### Install Start ######

install(
  DIRECTORY ${SY_SDK_LIBS_DIR}/
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  camera_config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  script/opencv_rgb_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/opencv_rgbIR_viewer.py
  DESTINATION lib/${PROJECT_NAME}
)

###### Install End ######


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
