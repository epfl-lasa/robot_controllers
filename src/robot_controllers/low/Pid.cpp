#include "robot_controllers/low/Pid.hpp"

namespace robot_controllers {
    namespace low {
        bool Pid::Init()
        {
            pid_params_.FromRobotParams(params_);
            curr_state_.position_ = Eigen::VectorXd::Zero(params_.input_dim_);

            return true;
        }

        void Pid::Update(const RobotState& state)
        {
            Eigen::VectorXd old_position(curr_state_.position_),
                intergral_error = (state.position_ + old_position) * pid_params_.time_step_ / 2;

            curr_state_ = state;

            output_.desired_.force_ = -pid_params_.p_matrix_ * (state.position_ - input_.desired_.position_)
                - pid_params_.d_matrix_ * (state.velocity_ - input_.desired_.velocity_)
                - pid_params_.i_matrix_ * intergral_error;
        }

        void Pid::SetParams(const ParamsPid& params)
        {
            pid_params_ = params;
            params_ = pid_params_.ToRobotParams();
        }

    } // namespace low
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(PidController, robot_controllers::low::Pid, "RobotControllers.AbstractController/1.0")
