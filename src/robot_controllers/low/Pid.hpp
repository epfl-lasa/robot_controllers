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
#ifndef ROBOT_CONTROLLERS_LOW_PID_HPP
#define ROBOT_CONTROLLERS_LOW_PID_HPP

#include <robot_controllers/AbstractController.hpp>
#include <vector>

namespace robot_controllers {
    namespace low {
        struct ParamsPid {
            Eigen::MatrixXd p_matrix_,
                d_matrix_,
                i_matrix_;
            double time_step_;

            RobotParams ToRobotParams() const;
            void FromRobotParams(const RobotParams& p);
        };

        class Pid : public AbstractController {
        public:
            explicit Pid(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin)
            {
                input_ = RobotIO(IOType::Position | IOType::Velocity);
                output_ = RobotIO(IOType::Force);
            }

            Pid(const unsigned int input_dim, const unsigned int output_dim, const double time_step) : AbstractController(IOType::Position | IOType::Velocity, IOType::Force)
            {
                params_.input_dim_ = input_dim;
                params_.output_dim_ = output_dim;
                params_.time_step_ = time_step;
            }

            ~Pid() {}

            bool Init() override;

            void Update(const RobotState& state) override;

            void SetParams(const ParamsPid& params);

        protected:
            ParamsPid pid_params_;
            Eigen::VectorXd intergral_error_;
            bool has_orientation_, has_position_, has_angular_velocity_, has_velocity_;
            unsigned int dim_;
        };

    } // namespace low

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_LOW_PID_HPP