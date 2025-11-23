cmake_minimum_required(VERSION 3.8)
project(sim_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)

add_executable(turtle_tf2_message_filter src/turtle_tf2_message_filter.cpp)
ament_target_dependencies(
  turtle_tf2_message_filter
  geometry_msgs
  message_filters
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)
if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(turtle_tf2_message_filter PUBLIC -DTF2_CPP_HEADERS)
endif()
install(TARGETS
  turtle_tf2_message_filter
  DESTINATION lib/${PROJECT_NAME})

add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
  static_turtle_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
)
install(TARGETS
  static_turtle_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

add_executable(turtle_tf2_broadcaster src/turtle_tf2_broadcaster.cpp)
ament_target_dependencies(
  turtle_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim)
install(TARGETS 
  turtle_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

add_executable(turtle_tf2_listener src/turtle_tf2_listener.cpp)
ament_target_dependencies(
  turtle_tf2_listener
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)
install(TARGETS 
  turtle_tf2_listener
  DESTINATION lib/${PROJECT_NAME})

add_executable(fixed_frame_tf2_broadcaster src/fixed_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
  fixed_frame_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
)
install(TARGETS 
  fixed_frame_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

add_executable(dynamic_frame_tf2_broadcaster src/dynamic_frame_tf2_broadcaster.cpp)
ament_target_dependencies(
  dynamic_frame_tf2_broadcaster
  geometry_msgs
  rclcpp
  tf2_ros
)
install(TARGETS 
  dynamic_frame_tf2_broadcaster
  DESTINATION lib/${PROJECT_NAME})

add_executable(turtle_tf2_listener_debug src/turtle_tf2_listener_debug.cpp)
ament_target_dependencies(
  turtle_tf2_listener_debug
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  turtlesim
)
install(TARGETS 
  turtle_tf2_listener_debug
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  

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

find_package(Doxygen)
if ( DOXYGEN_FOUND )
  set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_EXTRACT_ALL          YES )
  set( DOXYGEN_CLASS_DIAGRAMS       YES )
  set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
  set( DOXYGEN_HAVE_DOT             YES )
  set( DOXYGEN_CLASS_GRAPH          YES )
  set( DOXYGEN_CALL_GRAPH           YES )
  set( DOXYGEN_CALLER_GRAPH         YES )
  set( DOXYGEN_COLLABORATION_GRAPH  YES )
  set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
  set( DOXYGEN_EXTRACT_PRIVATE      YES )
  set( DOXYGEN_EXTRACT_PACKAGE      YES )
  set( DOXYGEN_EXTRACT_STATIC       YES )
  set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
  set( DOXYGEN_UML_LOOK             YES )
  set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
  set( DOXYGEN_TEMPLATE_RELATIONS   YES )
  set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
  set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
  set( DOXYGEN_DOT_TRANSPARENT      YES )
  set( DOXYGEN_DOT_IMAGE_FORMAT "pdf")
  set( DOXYGEN_GENERATE_LATEX YES)
  #make -C docs/latex/ pdf to make pdf
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

#
# Create the compilation database for clangd
#
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json




# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src/sm_2d/src
  # ${PROJECT_SOURCE_DIR}/libs
  )