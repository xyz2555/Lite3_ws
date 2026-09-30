cmake_minimum_required(VERSION 3.8)

project(lite3_dynamics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(lite3_kinematics REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

pkg_check_modules(GZ_TRANSPORT REQUIRED IMPORTED_TARGET gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED IMPORTED_TARGET gz-msgs10)

# ============================================================
# Dynamics library
# ============================================================

add_library(lite3_dynamics_lib
  src/dynamics.cpp
  # src/lite3_single_leg_dynamics.cpp
)

target_include_directories(lite3_dynamics_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

# ============================================================
# Gravity validator
# ============================================================

add_executable(gravity_validator
  src/gravity_validator.cpp
)

target_link_libraries(
  gravity_validator
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(mass_matrix_validator
  src/mass_matrix_validator.cpp
)

target_link_libraries(
  mass_matrix_validator
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(mg_torque_analyzer
  src/mg_torque_analyzer.cpp
)

target_link_libraries(
  mg_torque_analyzer
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(coriolis_validator
  src/coriolis_validator.cpp
)

target_link_libraries(
  coriolis_validator
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(batch_inverse_dynamics
  src/batch_inverse_dynamics.cpp
)

target_link_libraries(
  batch_inverse_dynamics
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(actuator_feasibility
  src/actuator_feasibility.cpp
)

target_link_libraries(
  actuator_feasibility
  Eigen3::Eigen
)

add_executable(
  contact_force_analyzer
  src/contact_force_analyzer.cpp
)

target_link_libraries(
  contact_force_analyzer
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(
  full_body_static_equilibrium
  src/full_body_static_equilibrium.cpp
)

target_link_libraries(
  full_body_static_equilibrium
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(contact_torque_validator
  src/contact_torque_validator.cpp
)

ament_target_dependencies(
  contact_torque_validator
  rclcpp
  sensor_msgs
  geometry_msgs
  ros_gz_interfaces
  lite3_kinematics
)

target_link_libraries(
  contact_torque_validator
  lite3_dynamics_lib
)

install(
  TARGETS contact_torque_validator
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(lite3_base_pose_bridge
  src/lite3_base_pose_bridge.cpp
)

ament_target_dependencies(
  lite3_base_pose_bridge
  rclcpp
  geometry_msgs
)

target_link_libraries(
  lite3_base_pose_bridge
  PkgConfig::GZ_TRANSPORT
  PkgConfig::GZ_MSGS
)

add_executable(
  stair_contact_equilibrium
  src/stair_contact_equilibrium.cpp
)

target_link_libraries(
  stair_contact_equilibrium
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

ament_target_dependencies(stair_contact_equilibrium
  Eigen3
)

add_executable(
  stair_posture_ik
  src/stair_posture_ik.cpp
)

target_link_libraries(
  stair_posture_ik
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(stair_contact_aware_inverse_dynamics
  src/stair_contact_aware_inverse_dynamics.cpp
)

target_link_libraries(stair_contact_aware_inverse_dynamics
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
)

ament_target_dependencies(
  stair_contact_aware_inverse_dynamics
  Eigen3
)

add_executable(stair_com_shift_contact_equilibrium
  src/stair_com_shift_contact_equilibrium.cpp
)

target_link_libraries(stair_com_shift_contact_equilibrium
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
)

ament_target_dependencies(
  stair_com_shift_contact_equilibrium
  Eigen3
)

add_executable(dynamic_inverse_dynamics_validator
  src/dynamic_inverse_dynamics_validator.cpp
)

ament_target_dependencies(
  dynamic_inverse_dynamics_validator
  rclcpp
  sensor_msgs
  rosgraph_msgs
)

target_link_libraries(
  dynamic_inverse_dynamics_validator
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(dynamic_inverse_dynamics_validator_qpoly_compare
  src/dynamic_inverse_dynamics_validator_qpoly_compare.cpp
)

ament_target_dependencies(
  dynamic_inverse_dynamics_validator_qpoly_compare
  rclcpp
  sensor_msgs
  rosgraph_msgs
  lite3_kinematics
)

target_link_libraries(
  dynamic_inverse_dynamics_validator_qpoly_compare
  lite3_dynamics_lib
)

add_executable(lite3_raw_joint_logger
  src/lite3_raw_joint_logger.cpp
)

ament_target_dependencies(
  lite3_raw_joint_logger
  rclcpp
  sensor_msgs
  rosgraph_msgs
)

target_link_libraries(
  lite3_raw_joint_logger
  lite3_dynamics_lib
)

add_executable(lite3_raw_clock_joint_logger
  src/lite3_raw_clock_joint_logger.cpp
)

ament_target_dependencies(
  lite3_raw_clock_joint_logger
  rclcpp
  sensor_msgs
  rosgraph_msgs
)

target_link_libraries(
  lite3_raw_clock_joint_logger
  lite3_dynamics_lib
)

add_executable(lite3_dynamic_inverse_dynamics_offline
  src/lite3_dynamic_inverse_dynamics_offline.cpp
)

ament_target_dependencies(
  lite3_dynamic_inverse_dynamics_offline
  lite3_kinematics
)

target_link_libraries(
  lite3_dynamic_inverse_dynamics_offline
  lite3_dynamics_lib
)

add_executable(lite3_high_accel_trajectory
  src/lite3_high_accel_trajectory.cpp
)

ament_target_dependencies(
  lite3_high_accel_trajectory
  rclcpp
  std_msgs
)

target_link_libraries(
  lite3_high_accel_trajectory
  lite3_dynamics_lib
)

add_executable(lite3_highacc_inverse_dynamics_window_compare
  src/lite3_highacc_inverse_dynamics_window_compare.cpp
)

ament_target_dependencies(lite3_highacc_inverse_dynamics_window_compare
  rclcpp
)

target_link_libraries(lite3_highacc_inverse_dynamics_window_compare
  Eigen3::Eigen
  lite3_dynamics_lib
)

add_executable(lite3_effort_gravity_excitation
  src/lite3_effort_gravity_excitation.cpp
)

ament_target_dependencies(lite3_effort_gravity_excitation
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(lite3_effort_gravity_excitation
  lite3_dynamics_lib
  lite3_kinematics::lite3_kinematics_lib
  Eigen3::Eigen
)

# ============================================================
# Install
# ============================================================

install(
  TARGETS
    lite3_dynamics_lib
    gravity_validator
    mass_matrix_validator
    mg_torque_analyzer
    coriolis_validator  
    batch_inverse_dynamics
    actuator_feasibility
    contact_force_analyzer
    full_body_static_equilibrium
    stair_contact_equilibrium 
    stair_posture_ik
    stair_contact_aware_inverse_dynamics
    stair_com_shift_contact_equilibrium
    lite3_base_pose_bridge
    dynamic_inverse_dynamics_validator
    dynamic_inverse_dynamics_validator_qpoly_compare
    lite3_raw_joint_logger
    lite3_raw_clock_joint_logger
    lite3_dynamic_inverse_dynamics_offline
    lite3_high_accel_trajectory
    lite3_highacc_inverse_dynamics_window_compare
    lite3_effort_gravity_excitation

  EXPORT lite3_dynamics_targets

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  EXPORT lite3_dynamics_targets
  FILE lite3_dynamicsTargets.cmake
  NAMESPACE lite3_dynamics::
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ============================================================
# Export
# ============================================================

ament_export_targets(lite3_dynamics_targets HAS_LIBRARY_TARGET)

ament_export_include_directories(include)

ament_export_dependencies(
  Eigen3
  lite3_kinematics
)

ament_package()