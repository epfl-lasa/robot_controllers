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
#ifndef ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP
#define ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP

#include <Eigen/Core>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    namespace high {
        struct ParamsLinearDS {
            Eigen::MatrixXd A_;
            double time_step_;

            RobotParams ToRobotParams() const;
            void FromRobotParams(const RobotParams& p);
        };

        class LinearDS : public AbstractController {
        public:
            explicit LinearDS(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin)
            {
                input_ = RobotIO(IOType::Position);
                output_ = RobotIO(IOType::Velocity | IOType::Position);
            }

            LinearDS() : AbstractController(IOType::Position, IOType::Velocity | IOType::Position) {}
            ~LinearDS() {}

            bool Init() override;

            void SetIOTypes(IOTypes input_type, IOTypes output_type) override {} // Do not allow changes in the IO types

            void Update(const RobotState& state) override;

            void SetParams(const ParamsLinearDS& params);

        protected:
            ParamsLinearDS linear_ds_params_;
        };
    } // namespace high
} // namespace robot_controllers

#endif