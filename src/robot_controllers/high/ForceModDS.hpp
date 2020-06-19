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
#ifndef ROBOT_CONTROLLERS_HIGH_FORCE_MOD_DS_HPP
#define ROBOT_CONTROLLERS_HIGH_FORCE_MOD_DS_HPP

#include <robot_controllers/AbstractController.hpp>
#include <vector>

namespace robot_controllers {
    namespace high {
        class ForceModDS : public AbstractController {
        public:
            explicit ForceModDS(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin)
            {
                input_ = RobotIO(IOType::Force);
                output_ = RobotIO(IOType::Velocity);
            }

            ForceModDS() : AbstractController(IOType::Force, IOType::Velocity) {}
            ~ForceModDS() {}

            bool Init() override;

            void SetIOTypes(IOTypes input_type, IOTypes output_type) override {} // Do not allow changes in the IO types

            void Update(const RobotState&) override;

            void SetDamping(double damping);

        protected:
            double damping_;
        };
    } // namespace high
} // namespace robot_controllers

#endif