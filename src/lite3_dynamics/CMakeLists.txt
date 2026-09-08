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

# ============================================================
# Dynamics library
# ============================================================

add_library(lite3_dynamics_lib
  src/dynamics.cpp
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

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ============================================================
# Export
# ============================================================

ament_export_include_directories(include)

ament_export_dependencies(
  Eigen3
  lite3_kinematics
)

ament_package()