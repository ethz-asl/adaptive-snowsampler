cmake_minimum_required(VERSION 3.8)
project(adaptive_snowsampler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

find_package(tf2_ros REQUIRED)

include_directories(
  include
)

add_executable(adaptive_snowsampler
  src/main.cpp
  src/adaptive_snowsampler.cpp)
ament_target_dependencies(adaptive_snowsampler rclcpp std_msgs geometry_msgs px4_msgs tf2 tf2_ros)
target_include_directories(adaptive_snowsampler PUBLIC "${Eigen3_INCLUDE_DIRS}")
target_link_libraries(adaptive_snowsampler)

install(
  TARGETS
  adaptive_snowsampler
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# copy the geotif files to the build directory
install(DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources/)