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
cmake_minimum_required(VERSION 3.0)

cmake_policy(SET CMP0057 NEW)

# Set the name of your project here
project(robot_controllers)

# When enabling SIMD, both the dependencies (e.g. RBDyn) and subsequent code need to be compiled with SIMD
option(ENABLE_SIMD "Build with all SIMD instructions on the current local machine" OFF)

find_package(PkgConfig)

# Find RBDyn library and dependencies
pkg_search_module(Eigen3 REQUIRED eigen3)

# Find Corrade
find_package(Corrade REQUIRED Utility PluginManager)

set(ROBOT_CONTROLLERS_BINARY_INSTALL_DIR bin)
set(ROBOT_CONTROLLERS_LIBRARY_INSTALL_DIR lib${LIB_SUFFIX})
set(ROBOT_CONTROLLERS_INCLUDE_INSTALL_DIR include/robot_controllers)

function(add_simd TARGET)
  if(ENABLE_SIMD)
    if(CMAKE_COMPILER_IS_GNUCXX)
      execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
      set(CXX_COMPILER_VERSION ${GCC_VERSION})
      target_compile_options(${TARGET} PUBLIC -march=native)
      if(GCC_VERSION VERSION_GREATER 7.0)
        target_compile_options(${TARGET} PUBLIC -faligned-new)
      endif()
    elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
      target_compile_options(${TARGET} PUBLIC -march=native -faligned-new)
    endif()
  endif()
endfunction()

add_subdirectory(src)
