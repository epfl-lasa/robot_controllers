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
# All headers for installation
set(low_level_controllers_HEADERS
    PassiveDS.hpp
    Pid.hpp
)

# PID plugin
corrade_add_plugin(PidController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf Pid.cpp ${robot_controllers_utils_SOURCES})

# Includes
target_include_directories(PidController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Eigen3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET PidController PROPERTY CXX_STANDARD 11)
set_property(TARGET PidController PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(PidController PUBLIC RobotControllers)

# Add SIMD
add_simd(PidController)

install(TARGETS PidController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

# PassiveDS plugin
corrade_add_plugin(PassiveDSController ${CMAKE_CURRENT_BINARY_DIR} "" Dummy.conf PassiveDS.cpp ${robot_controllers_utils_SOURCES})

# Includes
target_include_directories(PassiveDSController PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Eigen3_INCLUDE_DIRS}
    $<TARGET_PROPERTY:Corrade::PluginManager,INTERFACE_INCLUDE_DIRECTORIES>)

# Require C++11
set_property(TARGET PassiveDSController PROPERTY CXX_STANDARD 11)
set_property(TARGET PassiveDSController PROPERTY CXX_STANDARD_REQUIRED ON)

target_link_libraries(PassiveDSController PUBLIC RobotControllers)

# Add SIMD
add_simd(PassiveDSController)

install(TARGETS PassiveDSController
    LIBRARY DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR})

install(FILES ${low_level_controllers_HEADERS} DESTINATION ${ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR}/low)
install(FILES Dummy.conf DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR} RENAME PidController.conf)
install(FILES Dummy.conf DESTINATION ${ROBOT_CONTROLLERS_PLUGINS_DIR} RENAME PassiveDSController.conf)