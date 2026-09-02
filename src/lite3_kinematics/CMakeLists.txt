cmake_minimum_required(VERSION 3.8)

project(lite3_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(lite3_kinematics_lib
  src/kinematics.cpp
)

target_include_directories(lite3_kinematics_lib
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(lite3_kinematics_lib
  Eigen3::Eigen
)

add_executable(ik_validator
  src/ik_validator.cpp
)

target_link_libraries(ik_validator
  lite3_kinematics_lib
)

install(
  TARGETS
    lite3_kinematics_lib
    ik_validator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()