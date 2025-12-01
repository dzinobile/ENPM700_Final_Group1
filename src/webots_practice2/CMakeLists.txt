cmake_minimum_required(VERSION 3.8)
project(webots_practice2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(webots_ros2_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

# add_executable(publisher src/simple_publisher.cpp)
# ament_target_dependencies(publisher rclcpp geometry_msgs)
# install(TARGETS publisher DESTINATION lib/${PROJECT_NAME})

# add_executable(pursuer src/pursuer.cpp)
# ament_target_dependencies(pursuer rclcpp webots_ros2_msgs geometry_msgs nav_msgs tf2)
# install(TARGETS pursuer DESTINATION lib/${PROJECT_NAME})

add_executable(sheep_broadcaster src/sheep_broadcaster.cpp)
ament_target_dependencies(sheep_broadcaster
  rclcpp
  tf2_ros
  geometry_msgs
)

install(TARGETS
  sheep_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sheep_nav_trigger src/sheep_nav_trigger.cpp)
ament_target_dependencies(sheep_nav_trigger
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
)

install(TARGETS
  sheep_nav_trigger
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(nav_goal_sender src/nav_goal_sender.cpp)
ament_target_dependencies(nav_goal_sender rclcpp geometry_msgs nav2_msgs rclcpp_action)
install(TARGETS nav_goal_sender DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
)
install(
  DIRECTORY  launch worlds resource
  DESTINATION share/${PROJECT_NAME}
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
