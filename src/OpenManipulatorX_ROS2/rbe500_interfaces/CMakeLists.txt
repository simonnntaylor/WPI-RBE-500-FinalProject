cmake_minimum_required(VERSION 3.8)
project(rbe500_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/JointToEEVelocity.srv"
  "srv/EEToJointVelocity.srv"
)

ament_package()
