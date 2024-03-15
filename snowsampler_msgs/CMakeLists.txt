cmake_minimum_required(VERSION 2.8.3)
project(snowsampler_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

include_directories(include)

add_service_files(
  FILES
  Trigger.srv
  SetAngle.srv
  SetMaxSpeed.srv
  GetPosition.srv
  TakeMeasurement.srv
  SetMeasurementDepth.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs)
