#include <cassert>

#include "robot_controllers/high/LinearDS.hpp"

namespace robot_controllers {
    namespace high {
        RobotParams ParamsLinearDS::ToRobotParams() const
        {
            RobotParams p;

            p.input_dim_ = A_.rows();
            p.output_dim_ = A_.cols();
            p.time_step_ = time_step_;

            unsigned int size = A_.size();

            if (size > 0) {
                p.values_.resize(size);

                Eigen::MatrixXd::Map(p.values_.data(), p.output_dim_, p.input_dim_) = A_;
            }

            return p;
        }

        void ParamsLinearDS::FromRobotParams(const RobotParams& p)
        {
            if (p.input_dim_ == 0 || p.output_dim_ == 0)
                return;

            time_step_ = p.time_step_;

            A_ = Eigen::MatrixXd::Zero(p.output_dim_, p.input_dim_);

            unsigned int size = p.values_.size();
            // if only one element
            if (size == 1) {
                A_.diagonal() = Eigen::VectorXd::Constant(p.output_dim_, p.values_[0]);
            }
            else if (size == p.input_dim_) { // diagonal elements
                A_.diagonal() = Eigen::VectorXd::Map(p.values_.data(), p.output_dim_);
            }
            else { // full matrix
                A_ = Eigen::MatrixXd::Map(p.values_.data(), p.output_dim_, p.input_dim_);
            }
        }

        bool LinearDS::Init()
        {
            linear_ds_params_.FromRobotParams(params_);

            input_.desired_.position_ = Eigen::VectorXd::Zero(params_.input_dim_);
            return true;
        }

        void LinearDS::Update(const RobotState& state)
        {
            output_.desired_.velocity_ = linear_ds_params_.A_ * (input_.desired_.position_ - state.position_);
            output_.desired_.position_ = state.position_ + params_.time_step_ * output_.desired_.velocity_;
        }

        void LinearDS::SetParams(const ParamsLinearDS& params)
        {
            linear_ds_params_ = params;
            params_ = linear_ds_params_.ToRobotParams();
        }
    } // namespace high
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(LinearDSController, robot_controllers::high::LinearDS, "RobotControllers.AbstractController/1.0")