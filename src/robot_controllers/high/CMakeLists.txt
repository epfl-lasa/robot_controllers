#|
#|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
#|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
#|              Bernardo Fichera
#|    email:   konstantinos.chatzilygeroudis@epfl.ch
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
# All headers for installation
set(high_level_controllers_HEADERS
    LinearDS.hpp
)

# Linear DS plugin
corrade_add_plugin(LinearDSController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf LinearDS.cpp ${robot_controllers_utils_SOURCES})

# Includes
target_include_directories(LinearDSController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Eigen3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET LinearDSController PROPERTY CXX_STANDARD 11)
set_property(TARGET LinearDSController PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(LinearDSController PUBLIC RobotControllers)

# Add SIMD
add_simd(LinearDSController)

install(TARGETS LinearDSController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

install(FILES ${high_level_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/high)