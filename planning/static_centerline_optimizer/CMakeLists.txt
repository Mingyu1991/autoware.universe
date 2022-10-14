cmake_minimum_required(VERSION 3.5)
project(static_centerline_optimizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

rosidl_generate_interfaces(
  static_centerline_optimizer
  "srv/LoadMap.srv"
  "srv/PlanRoute.srv"
  "srv/PlanPath.srv"
  DEPENDENCIES builtin_interfaces
)

ament_auto_add_executable(static_centerline_optimizer_node
  src/main.cpp
  src/static_centerline_optimizer_node.cpp
  src/optimization_node.cpp
  src/utils.cpp
  )

target_include_directories(static_centerline_optimizer_node
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(static_centerline_optimizer_node
  ${OpenCV_LIBRARIES}
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(static_centerline_optimizer_node
    static_centerline_optimizer "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target static_centerline_optimizer "rosidl_typesupport_cpp")
    target_link_libraries(static_centerline_optimizer_node "${cpp_typesupport_target}")
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    rviz
)

install(PROGRAMS
  scripts/optimize_centerline.sh
  DESTINATION lib/${PROJECT_NAME}
)

#rclcpp_components_register_node(trajectory_analyzer_node
#  PLUGIN "planning_debug_tools::TrajectoryAnalyzerNode"
#  EXECUTABLE trajectory_analyzer_exe
#)
#
#rclcpp_components_register_node(stop_reason_visualizer_node
#  PLUGIN "planning_debug_tools::StopReasonVisualizerNode"
#  EXECUTABLE stop_reason_visualizer_exe
#)
#
#ament_auto_package(
#  INSTALL_TO_SHARE
#    launch
#)
#
#install(PROGRAMS
#  scripts/trajectory_visualizer.py
#  scripts/closest_velocity_checker.py
#  DESTINATION lib/${PROJECT_NAME}
#)



#cmake_minimum_required(VERSION 3.5)
#project(static_centerline_optimizer)
#
#find_package(ament_cmake_auto REQUIRED)
#ament_auto_find_build_dependencies()
#
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/LoadMap.srv"
#  "srv/PlanRoute.srv"
#  "srv/PlanPath.srv"
#  DEPENDENCIES
#    std_msgs
#    )
