#include <cassert>

#include "robot_controllers/high/LinearDS.hpp"

namespace robot_controllers {
    namespace high {
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