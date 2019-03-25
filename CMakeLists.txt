cmake_minimum_required(VERSION 3.0)

# Set the name of your project here
project(robot_controllers)

find_package(PkgConfig)

# Find RBDyn library and dependencies
pkg_search_module(Eigen3 REQUIRED eigen3)

# Find Corrade
find_package(Corrade REQUIRED Utility PluginManager)

set(ROBOT_CONTROLLERS_BINARY_INSTALL_DIR bin)
set(ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR lib${LIB_SUFFIX})
set(ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR include/robot_controllers)

add_subdirectory(src)