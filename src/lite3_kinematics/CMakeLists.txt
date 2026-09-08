cmake_minimum_required(VERSION 3.8)

project(lite3_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================
# Kinematics library
# ============================================================

add_library(lite3_kinematics_lib
  src/kinematics.cpp
)

target_include_directories(lite3_kinematics_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  lite3_kinematics_lib
  Eigen3::Eigen
)

# ============================================================
# Validators / analyzers
# ============================================================

add_executable(ik_validator
  src/ik_validator.cpp
)

target_link_libraries(
  ik_validator
  lite3_kinematics_lib
)

add_executable(workspace_generator
  src/workspace_generator.cpp
)

target_link_libraries(
  workspace_generator
  lite3_kinematics_lib
)

add_executable(single_leg_stair_analyzer
  src/single_leg_stair_analyzer.cpp
)

target_link_libraries(
  single_leg_stair_analyzer
  lite3_kinematics_lib
)

add_executable(jacobian_validator
  src/jacobian_validator.cpp
)

target_link_libraries(
  jacobian_validator
  lite3_kinematics_lib
)

add_executable(smooth_stair_trajectory_analyzer
  src/smooth_stair_trajectory_analyzer.cpp
)

target_link_libraries(
  smooth_stair_trajectory_analyzer
  lite3_kinematics_lib
)

# ============================================================
# Install library ONLY as exported target
# ============================================================

install(
  TARGETS
    lite3_kinematics_lib

  EXPORT
    export_lite3_kinematics

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================
# Install validator executables
# They are NOT exported as library targets.
# ============================================================

install(
  TARGETS
    ik_validator
    workspace_generator
    single_leg_stair_analyzer
    jacobian_validator
    smooth_stair_trajectory_analyzer

  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Install headers
# ============================================================

install(
  DIRECTORY include/
  DESTINATION include
)

# ============================================================
# Export library and include directory
# ============================================================

ament_export_include_directories(include)

ament_export_targets(
  export_lite3_kinematics
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(Eigen3)

ament_package()