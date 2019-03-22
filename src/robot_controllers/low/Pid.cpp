#include "robot_controllers/low/Pid.hpp"

namespace robot_controllers {
    namespace low {
        bool Pid::Init()
        {
            params_.p_matrix_ = Eigen::MatrixXd::Zero(params_.input_dim_, params_.output_dim_);
            params_.d_matrix_ = Eigen::MatrixXd::Zero(params_.input_dim_, params_.output_dim_);
            params_.i_matrix_ = Eigen::MatrixXd::Zero(params_.input_dim_, params_.output_dim_);

            return true;
        }

        void Pid::Update(const RobotState& state)
        {
            Eigen::VectorXd old_position(curr_state_.position_),
                intergral_error = (state.position_ + old_position) * params_.time_step_ / 2;

            curr_state_ = state;

            output_.desired_.force_ = -params_.p_matrix_ * (state.position_ - input_.desired_.position_)
                - params_.d_matrix_ * (state.velocity_ - input_.desired_.velocity_)
                - params_.i_matrix_ * intergral_error;
        }

        void Pid::SetParams(const Eigen::MatrixXd& p_matrix, const Eigen::MatrixXd& d_matrix, const Eigen::MatrixXd& i_matrix)
        {
            params_.p_matrix_ = p_matrix;
            params_.d_matrix_ = d_matrix;
            params_.i_matrix_ = i_matrix;
            curr_state_.position_ = Eigen::VectorXd::Zero(p_matrix.rows());
        }

    } // namespace low
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(PidController, robot_controllers::low::Pid, "RobotControllers.AbstractController/1.0")
