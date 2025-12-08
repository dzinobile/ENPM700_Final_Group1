cmake_minimum_required(VERSION 3.8)
project(sheepdog)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================
# Core dependencies (required)
# ========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ========================
# Optional dependencies (CI-safe)
# ========================
find_package(webots_ros2_driver QUIET)
find_package(webots_ros2_msgs QUIET)
find_package(Doxygen QUIET)
find_package(catch_ros2 QUIET)

if(webots_ros2_driver_FOUND)
  add_definitions(-DHAVE_WEBOTS)
  message(STATUS "webots_ros2_driver found — Webots enabled")
else()
  message(WARNING "webots_ros2_driver NOT found — Webots disabled")
endif()

if(DOXYGEN_FOUND)
  message(STATUS "Doxygen found — docs enabled")
else()
  message(WARNING "Doxygen NOT found — docs disabled")
endif()

include_directories(include)

# ========================
# Main node
# ========================
add_executable(sheepdog_node
  src/main.cpp
  src/SheepdogNode.cpp
  src/States.cpp
  src/ExploreState.cpp
  src/EncircleState.cpp
  src/HerdState.cpp
  src/DoneState.cpp)

ament_target_dependencies(sheepdog_node
  rclcpp
  rclcpp_action
  nav2_msgs
  sensor_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs)

# Only link Webots if present
if(webots_ros2_driver_FOUND)
  ament_target_dependencies(sheepdog_node
    webots_ros2_driver
    webots_ros2_msgs
  )
endif()

target_include_directories(sheepdog_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS sheepdog_node
  DESTINATION lib/${PROJECT_NAME})

# ========================
# Sheep broadcaster
# ========================
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

# ========================
# Resource install
# ========================
ament_export_include_directories(include)

install(
  DIRECTORY launch worlds resource
  DESTINATION share/${PROJECT_NAME}
)

# ========================
# Tests (optional in CI)
# ========================
if(BUILD_TESTING AND catch_ros2_FOUND)
  message(STATUS "catch_ros2 found — enabling integration tests")

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
    rclcpp std_msgs rclcpp_action nav2_msgs tf2_ros geometry_msgs
  )

  install(TARGETS integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  catch_ros2_add_integration_test(
    ExampleIntegration_TestYAML
    LAUNCH_FILE integration_test.launch.yaml
  )
elseif(BUILD_TESTING)
  message(WARNING "catch_ros2 not found — skipping integration tests")
endif()

# ========================
# Doxygen (optional in CI)
# ========================
if(DOXYGEN_FOUND)
  set(DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs)
  set(DOXYGEN_COLLABORATION_GRAPH  YES)
  set(DOXYGEN_EXTRACT_ALL          YES)
  set(DOXYGEN_CLASS_DIAGRAMS       YES)
  set(DOXYGEN_HIDE_UNDOC_RELATIONS NO)
  set(DOXYGEN_HAVE_DOT             YES)
  set(DOXYGEN_CLASS_GRAPH          YES)
  set(DOXYGEN_CALL_GRAPH           YES)
  set(DOXYGEN_CALLER_GRAPH         YES)
  set(DOXYGEN_BUILTIN_STL_SUPPORT  YES)
  set(DOXYGEN_EXTRACT_PRIVATE      YES)
  set(DOXYGEN_EXTRACT_PACKAGE      YES)
  set(DOXYGEN_EXTRACT_STATIC       YES)
  set(DOXYGEN_EXTRACT_LOCALMETHODS YES)
  set(DOXYGEN_UML_LOOK             YES)
  set(DOXYGEN_UML_LIMIT_NUM_FIELDS 50)
  set(DOXYGEN_TEMPLATE_RELATIONS   YES)
  set(DOXYGEN_DOT_GRAPH_MAX_NODES  100)
  set(DOXYGEN_MAX_DOT_GRAPH_DEPTH  0)
  set(DOXYGEN_DOT_TRANSPARENT      YES)

  doxygen_add_docs(docs
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/test
  )
else()
  message(WARNING "Doxygen not found — skipping docs build")
endif()

# ========================
# Scripts
# ========================
install(PROGRAMS
  scripts/generate_coverage_report.bash
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
