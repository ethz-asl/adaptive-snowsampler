cmake_minimum_required(VERSION 3.14.4)
project(snowsampler_msgs)

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/Trigger.srv
  srv/SetAngle.srv
  srv/SetMaxSpeed.srv
  srv/GetPosition.srv
  srv/TakeMeasurement.srv
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()