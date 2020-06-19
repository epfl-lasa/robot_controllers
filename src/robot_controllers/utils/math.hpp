//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
//|              Bernardo Fichera
//|    email:   costashatz@gmail.com
//|             bernardo.fichera@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of robot_controllers.
//|
//|    robot_controllers is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    robot_controllers is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#ifndef ROBOT_CONTROLLERS_UTILS_MATH_HPP
#define ROBOT_CONTROLLERS_UTILS_MATH_HPP

#include <Eigen/Core>

namespace robot_controllers {
    namespace utils {
        Eigen::Vector3d rotation_error(const Eigen::Matrix3d& R_desired, const Eigen::Matrix3d& R_current);
    } // namespace utils
} // namespace robot_controllers

#endif