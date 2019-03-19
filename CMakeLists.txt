cmake_minimum_required(VERSION 3.0)

# Set the name of your project here
project(robot_controllers)

set (CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED)

set(ROBOT_CONTROLLERS_BINARY_INSTALL_DIR bin)
set(ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR lib${LIB_SUFFIX})
set(ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR include/robot_controllers)

add_subdirectory(src/robot_controllers)
add_subdirectory(src/examples)