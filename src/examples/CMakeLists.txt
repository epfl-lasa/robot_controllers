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
add_executable(linear_test linear_test.cpp)

# Require C++11
set_property(TARGET linear_test PROPERTY CXX_STANDARD 11)
set_property(TARGET linear_test PROPERTY CXX_STANDARD_REQUIRED ON)

# add_library(PassiveDS SHARED IMPORTED)
# set_property(TARGET PassiveDS PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/../robot_controllers/low/PassiveDSController.so)

# Add SIMD
add_simd(linear_test)

target_link_libraries(linear_test
  RobotControllers
  # PassiveDSController
)