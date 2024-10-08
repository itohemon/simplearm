cmake_minimum_required(VERSION 3.8)
project(simplearm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/pub_joint_state.cpp
  src/kinematic1.cpp
  src/inverted_kinematic.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_nodes(${PROJECT_NAME}
  "simplearm::PubJointState"
  "simplearm::Kinematic1"
  "simplearm::InvertedKinematic")

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
  )

ament_package()
