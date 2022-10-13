cmake_minimum_required(VERSION 3.14)
project(static_centerline_optimizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_executable(static_centerline_optimizer
  src/main.cpp
  src/static_centerline_optimizer_node.cpp
  src/optimization_node.cpp
  src/utils.cpp)

target_include_directories(static_centerline_optimizer
  SYSTEM PUBLIC
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(static_centerline_optimizer
  ${OpenCV_LIBRARIES}
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    rviz
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "srv/LoadMap.srv"
#   DEPENDENCIES
#     autoware_common_msgs
#     std_msgs
# )

install(PROGRAMS
  scripts/optimize_centerline.sh
  DESTINATION lib/${PROJECT_NAME}
)
