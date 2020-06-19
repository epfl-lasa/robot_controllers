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
#include "robot_controllers/low/Pid.hpp"
#include "robot_controllers/utils/math.hpp"

#include <Eigen/Dense>

namespace robot_controllers {
    namespace low {
        RobotParams ParamsPid::ToRobotParams() const
        {
            assert(p_matrix_.size() == d_matrix_.size() && p_matrix_.size() == i_matrix_.size());
            RobotParams p;

            p.input_dim_ = p_matrix_.rows();
            p.output_dim_ = p_matrix_.cols();
            p.time_step_ = time_step_;

            unsigned int size = p_matrix_.size() + d_matrix_.size() + i_matrix_.size();

            if (size > 0) {
                p.values_.resize(size);

                Eigen::MatrixXd::Map(p.values_.data(), p.output_dim_, p.input_dim_) = p_matrix_;
                Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size(), p.output_dim_, p.input_dim_) = d_matrix_;
                Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size() + d_matrix_.size(), p.output_dim_, p.input_dim_) = i_matrix_;
            }

            return p;
        }

        void ParamsPid::FromRobotParams(const RobotParams& p)
        {
            if (p.input_dim_ == 0 || p.output_dim_ == 0 || p.values_.size() < 2)
                return;

            time_step_ = p.time_step_;

            p_matrix_.resize(p.output_dim_, p.input_dim_);
            d_matrix_.resize(p.output_dim_, p.input_dim_);
            i_matrix_.resize(p.output_dim_, p.input_dim_);

            unsigned int size = p.values_.size();
            // <= 3 values: one for P, one for D, one for I (in that order)
            if (size <= 3) {
                p_matrix_ = Eigen::MatrixXd::Zero(p.output_dim_, p.input_dim_);
                d_matrix_ = Eigen::MatrixXd::Zero(p.output_dim_, p.input_dim_);
                i_matrix_ = Eigen::MatrixXd::Zero(p.output_dim_, p.input_dim_);

                p_matrix_.diagonal() = Eigen::VectorXd::Constant(p.output_dim_, p.values_[0]);
                d_matrix_.diagonal() = Eigen::VectorXd::Constant(p.output_dim_, p.values_[1]);
                if (size == 3) {
                    i_matrix_.diagonal() = Eigen::VectorXd::Constant(p.output_dim_, p.values_[2]);
                }
            }
            // diagonal elements only: for P, for D and for I (in that order)
            else if (size <= 3 * p.output_dim_) {
                p_matrix_ = Eigen::MatrixXd::Zero(p.output_dim_, p.input_dim_);
                d_matrix_ = Eigen::MatrixXd::Zero(p.output_dim_, p.input_dim_);
                i_matrix_ = Eigen::MatrixXd::Zero(p.output_dim_, p.input_dim_);

                p_matrix_.diagonal() = Eigen::VectorXd::Map(p.values_.data(), p.output_dim_);
                d_matrix_.diagonal() = Eigen::VectorXd::Map(p.values_.data() + p.output_dim_, p.output_dim_);
                if (size == 3 * p.output_dim_) {
                    i_matrix_.diagonal() = Eigen::VectorXd::Map(p.values_.data() + 2 * p.output_dim_, p.output_dim_);
                }
            }
            else { // full matrices
                p_matrix_ = Eigen::MatrixXd::Map(p.values_.data(), p.output_dim_, p.input_dim_);
                d_matrix_ = Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size(), p.output_dim_, p.input_dim_);
                i_matrix_ = Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size() + d_matrix_.size(), p.output_dim_, p.input_dim_);
            }
        }

        bool Pid::Init()
        {
            // sanity checks
            // if we have position input, output needs to have force
            if (((input_.GetType() & IOType::Position) || (input_.GetType() & IOType::Velocity)) && !(output_.GetType() & IOType::Force))
                return false;
            // if we have orientation input, output needs to have torque
            if (((input_.GetType() & IOType::Orientation) || (input_.GetType() & IOType::AngularVelocity)) && !(output_.GetType() & IOType::Torque))
                return false;

            pid_params_.FromRobotParams(params_);

            has_orientation_ = has_position_ = has_angular_velocity_ = has_velocity_ = false;

            if (input_.GetType() & IOType::Position)
                has_position_ = true;
            if (input_.GetType() & IOType::Velocity)
                has_velocity_ = true;
            if (input_.GetType() & IOType::Orientation)
                has_orientation_ = true;
            if (input_.GetType() & IOType::AngularVelocity)
                has_angular_velocity_ = true;

            dim_ = params_.input_dim_;

            if (has_orientation_ || has_angular_velocity_) {
                // Orientation is always 3D
                dim_ = params_.input_dim_ - 3;
            }

            if ((has_orientation_ || has_angular_velocity_) && (params_.input_dim_ < 3)) {
                // if we have orientation, then dimension of positions/orientations should be 3
                return false;
            }

            intergral_error_ = Eigen::VectorXd::Zero(params_.input_dim_);

            return true;
        }

        void Pid::Update(const RobotState& curr_state)
        {
            Eigen::VectorXd curr_pose = Eigen::VectorXd::Zero(params_.input_dim_);
            if (has_position_)
                curr_pose.tail(dim_) = curr_state.position_;
            if (has_orientation_) // Orientation goes first
                curr_pose.head(3) = curr_state.orientation_;

            Eigen::VectorXd desired_pose = Eigen::VectorXd::Zero(params_.input_dim_);
            if (has_position_)
                desired_pose.tail(dim_) = input_.desired_.position_;
            if (has_orientation_)
                desired_pose.head(3) = input_.desired_.orientation_;

            Eigen::VectorXd error = desired_pose - curr_pose;
            if (has_orientation_) {
                Eigen::Matrix3d curr_rot = Eigen::AngleAxisd(curr_state.orientation_.norm(), curr_state.orientation_.normalized()).toRotationMatrix();
                Eigen::Matrix3d desired_rot = Eigen::AngleAxisd(input_.desired_.orientation_.norm(), input_.desired_.orientation_.normalized()).toRotationMatrix();
                error.head(3) = utils::rotation_error(desired_rot, curr_rot);
            }

            Eigen::VectorXd vel_error = Eigen::VectorXd::Zero(params_.input_dim_);
            // if not velocity given, we assume target is zero velocities
            if (has_position_)
                vel_error.tail(dim_) = -curr_state.velocity_;
            if (has_velocity_)
                vel_error.tail(dim_) += input_.desired_.velocity_;

            if (has_orientation_)
                vel_error.head(3) = -curr_state.angular_velocity_;
            if (has_angular_velocity_)
                vel_error.head(3) += input_.desired_.angular_velocity_;

            intergral_error_.array() += error.array() * params_.time_step_; // TO-DO: What about the velocity error?

            Eigen::VectorXd ft = pid_params_.p_matrix_ * error + pid_params_.d_matrix_ * vel_error + pid_params_.i_matrix_ * intergral_error_;

            if (has_position_ || has_velocity_)
                output_.desired_.force_ = ft.tail(dim_);
            if (has_orientation_ || has_angular_velocity_)
                output_.desired_.torque_ = ft.head(3);
        }

        void Pid::SetParams(const ParamsPid& params)
        {
            pid_params_ = params;
            params_ = pid_params_.ToRobotParams();
        }

    } // namespace low
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(PidController, robot_controllers::low::Pid, "RobotControllers.AbstractController/1.0")
