cmake_minimum_required(VERSION 3.8)

project(lite3_description)

if(NOT_CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

# Install the complete Lite3 model directory.
# This preserves:
#   urdf/
#   urdf/meshes/
#   mjcf/
#   usd/

install(
    DIRECTORY Lite3
    DESTINATION share/${PROJECT_NAME}
)

ament_package()