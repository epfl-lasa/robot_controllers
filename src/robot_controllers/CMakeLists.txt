#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
#|              Bernardo Fichera
#|    email:   costashatz@gmail.com
#|             bernardo.fichera@epfl.ch
#|    website: lasa.epfl.ch
#|
#|    This file is part of robot_controllers.
#|
#|    robot_controllers is free software: you can redistribute it and/or modify
#|    it under the terms of the GNU General Public License as published by
#|    the Free Software Foundation, either version 3 of the License, or
#|    (at your option) any later version.
#|
#|    robot_controllers is distributed in the hope that it will be useful,
#|    but WITHOUT ANY WARRANTY; without even the implied warranty of
#|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#|    GNU General Public License for more details.
#|
set(robot_controllers_HEADERS
    AbstractController.hpp
    CascadeController.hpp
    SumController.hpp
    ${CMAKE_CURRENT_BINARY_DIR}/Config.hpp
)

set(robot_controllers_SOURCES
    AbstractController.cpp
    CascadeController.cpp
    SumController.cpp
)

set(robot_controllers_utils_HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/math.hpp
)

set(robot_controllers_utils_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/math.cpp
)

set(ROBOT_CONTROLLERS_PLUGINS_DIR ${CMAKE_INSTALL_PREFIX}/lib/RobotControllersPlugins)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/configure.h.cmake
               ${CMAKE_CURRENT_BINARY_DIR}/Config.hpp)

add_library(RobotControllers SHARED ${robot_controllers_SOURCES} ${robot_controllers_utils_SOURCES})

target_include_directories(RobotControllers PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Eigen3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>
    PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/..)

target_link_libraries(RobotControllers PUBLIC Corrade::PluginManager)

# Require C++11
set_property(TARGET RobotControllers PROPERTY CXX_STANDARD 11)
set_property(TARGET RobotControllers PROPERTY CXX_STANDARD_REQUIRED ON)

# Add SIMD
add_simd(RobotControllers)

install(TARGETS RobotControllers
    RUNTIME DESTINATION ${ROBOT_CONTROLLERS_BINARY_INSTALL_DIR}
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR}
    ARCHIVE DESTINATION ${ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR})

install(FILES ${robot_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR})
install(FILES ${robot_controllers_utils_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/utils)

add_subdirectory(low)
add_subdirectory(high)
