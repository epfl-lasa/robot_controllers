cmake_minimum_required(VERSION 3.0)

# Set the name of your project here
project("control_stack")

set (CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED)
find_package(Boost 1.58 REQUIRED)

set(CONTROL_STACK_BINARY_INSTALL_DIR bin)
set(CONTROL_STACK_LIBRARY_INSTALL_DIR lib${LIB_SUFFIX})
set(CONTROL_STACK_INCLUDE_INSTALL_DIR include/control_stack)

add_subdirectory(control_stack)
add_subdirectory(examples)