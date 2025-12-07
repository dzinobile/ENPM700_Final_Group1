cmake_minimum_required(VERSION 3.8)
project(sheepdog)

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
find_package(catch_ros2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


include_directories(include)

add_executable(sheepdog_node
  src/main.cpp
  src/SheepdogNode.cpp
  src/States.cpp
  src/ExploreState.cpp
  src/EncircleState.cpp
  src/HerdState.cpp)
ament_target_dependencies(sheepdog_node
  rclcpp
  rclcpp_action
  nav2_msgs
  sensor_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs)
target_include_directories(sheepdog_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
install(TARGETS sheepdog_node
  DESTINATION lib/${PROJECT_NAME})

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

# Integration test
if(BUILD_TESTING)
  # disable these built-in tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)


  add_executable(integration_test_node
    test/integration_test_node.cpp
    )

  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main "${cpp_typesupport_target}"
    )
  ament_target_dependencies(integration_test_node
    rclcpp std_msgs rclcpp_action nav2_msgs
    )
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )


  catch_ros2_add_integration_test (ExampleIntegration_TestYAML
    LAUNCH_FILE integration_test.launch.yaml  # example in YAML
    )


endif()

ament_package()
