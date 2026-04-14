cmake_minimum_required(VERSION 3.8)
project(tactilesensors4)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-ggdb -O3 -fPIC -march=native -mno-avx")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StaticData.msg"
  "msg/Dynamic.msg"
  "msg/Accelerometer.msg"
  "msg/Gyroscope.msg"
  "msg/Magnetometer.msg"
  "msg/Quaternion.msg"
  "msg/EulerAngle.msg"
  "srv/TactileSensors.srv"
  DEPENDENCIES std_msgs
)

include_directories(src)

add_executable(PollData4
  src/PollData4.cpp
  src/Communication.cpp
  src/Finger.cpp
)

ament_target_dependencies(PollData4
  rclcpp
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(PollData4 "${cpp_typesupport_target}")

install(TARGETS PollData4
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/SensorDouble.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
