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
#include <cassert>

#include "robot_controllers/high/ForceModDS.hpp"

namespace robot_controllers {
    namespace high {
        bool ForceModDS::Init()
        {
            assert(params_.input_dim_ == params_.output_dim_);
            assert(params_.values_.size() > 0);

            SetDamping(params_.values_[0]);

            return true;
        }

        void ForceModDS::Update(const RobotState&)
        {
            output_.desired_.velocity_ = input_.desired_.force_ / damping_;
        }

        void ForceModDS::SetDamping(double damping)
        {
            damping_ = damping;
            assert(damping_ > 0.);
        }
    } // namespace high
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(ForceModDSController, robot_controllers::high::ForceModDS, "RobotControllers.AbstractController/1.0")
