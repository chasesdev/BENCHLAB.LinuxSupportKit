cmake_minimum_required(VERSION 3.8)
project(benchlab_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  # Message files
  "msg/PowerSensor.msg"
  "msg/FanSensor.msg"
  "msg/BenchLabTelemetry.msg"
  "msg/DeviceInfo.msg"
  "msg/FanProfile.msg"
  "msg/RGBConfig.msg"
  "msg/CalibrationData.msg"

  # Service files
  "srv/SetDeviceName.srv"
  "srv/SetFanProfile.srv"
  "srv/SetRGB.srv"
  "srv/SetCalibration.srv"
  "srv/LoadCalibration.srv"
  "srv/StoreCalibration.srv"
  "srv/ExecuteAction.srv"

  DEPENDENCIES std_msgs
)

ament_package()
