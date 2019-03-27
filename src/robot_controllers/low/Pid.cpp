#include "robot_controllers/low/Pid.hpp"

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
