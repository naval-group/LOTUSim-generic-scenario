cmake_minimum_required(VERSION 3.8)
project(gz_ros2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lotusim_msgs REQUIRED)

## Add executables
add_executable(wind_ros_to_gz_bridge src/wind_ros_to_gz_bridge.cpp)
add_executable(stats_gz_to_ros_bridge src/stats_gz_to_ros_bridge.cpp)

## Link dependencies to wind_ros_to_gz_bridge
ament_target_dependencies(wind_ros_to_gz_bridge
  rclcpp
  lotusim_msgs
  geometry_msgs
)
target_link_libraries(wind_ros_to_gz_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

## Link dependencies to stats_gz_to_ros_bridge
ament_target_dependencies(stats_gz_to_ros_bridge
  rclcpp
  lotusim_msgs
)
target_link_libraries(stats_gz_to_ros_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

## Install both executables
install(TARGETS
  wind_ros_to_gz_bridge
  stats_gz_to_ros_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
