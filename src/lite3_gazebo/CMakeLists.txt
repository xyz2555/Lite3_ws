cmake_minimum_required(VERSION 3.8)
project(lite3_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lite3_kinematics REQUIRED)

add_executable(ik_gazebo_validator
  src/ik_gazebo_validator.cpp
)

ament_target_dependencies(
  ik_gazebo_validator
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(
  ik_gazebo_validator
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(cartesian_trajectory_validator
  src/cartesian_trajectory_validator.cpp
)

ament_target_dependencies(
  cartesian_trajectory_validator
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(
  cartesian_trajectory_validator
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)
add_executable(stair_trajectory_validator
  src/stair_trajectory_validator.cpp
)

ament_target_dependencies(
  stair_trajectory_validator
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(
  stair_trajectory_validator
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

install(
  TARGETS
    stair_trajectory_validator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    cartesian_trajectory_validator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS
    ik_gazebo_validator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/gazebo_fk_validator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()