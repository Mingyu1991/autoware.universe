cmake_minimum_required(VERSION 3.14)
project(planning_validator)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(planning_validator_helpers SHARED
  src/utils.cpp
  src/debug_marker.cpp
)

ament_auto_add_library(planning_validator_component SHARED
  include/planning_validator/planning_validator.hpp
  src/planning_validator.cpp
)
target_link_libraries(planning_validator_component planning_validator_helpers)

rclcpp_components_register_node(planning_validator_component
  PLUGIN "planning_validator::PlanningValidator"
  EXECUTABLE planning_validator_node
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/PlanningValidatorStatus.msg"
  DEPENDENCIES builtin_interfaces
)

# to use a message defined in the same package
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(planning_validator_component
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(planning_validator_component "${cpp_typesupport_target}")
endif()

if(BUILD_TESTING)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    launch
)
