cmake_minimum_required(VERSION 3.14)
project(traffic_light_optical_flow_based_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Boost 1.45 REQUIRED)

include_directories(
  SYSTEM
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_nodelet SHARED
  src/nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}_nodelet
  PLUGIN "traffic_light::TrafficLightOpticalFlowBasedDetectorNodelet"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
