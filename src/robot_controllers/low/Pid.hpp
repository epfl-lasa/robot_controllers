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

            RobotParams ToRobotParams() const
            {
                assert(p_matrix_.size() == d_matrix_.size() && p_matrix_.size() == i_matrix_.size());
                RobotParams p;

                p.input_dim_ = p_matrix_.rows();
                p.output_dim_ = p_matrix_.cols();
                p.time_step_ = time_step_;

                unsigned int size = p_matrix_.size() + d_matrix_.size() + i_matrix_.size();

                if (size > 0) {
                    p.values_.resize(size);

                    Eigen::MatrixXd::Map(p.values_.data(), p.input_dim_, p.output_dim_) = p_matrix_;
                    Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size(), p.input_dim_, p.output_dim_) = d_matrix_;
                    Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size() + d_matrix_.size(), p.input_dim_, p.output_dim_) = i_matrix_;
                }

                return p;
            }

            void FromRobotParams(const RobotParams& p)
            {
                if (p.input_dim_ == 0 || p.output_dim_ == 0)
                    return;

                time_step_ = p.time_step_;

                p_matrix_.resize(p.input_dim_, p.output_dim_);
                d_matrix_.resize(p.input_dim_, p.output_dim_);
                i_matrix_.resize(p.input_dim_, p.output_dim_);

                p_matrix_ = Eigen::MatrixXd::Map(p.values_.data(), p.input_dim_, p.output_dim_);
                d_matrix_ = Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size(), p.input_dim_, p.output_dim_);
                i_matrix_ = Eigen::MatrixXd::Map(p.values_.data() + p_matrix_.size() + d_matrix_.size(), p.input_dim_, p.output_dim_);
            }
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
                Init();
            }

            ~Pid() {}

            bool Init() override;

            void Update(const RobotState& state) override;

            void SetParams(const ParamsPid& params);

            // SetInput  -> Inherited from AbstractController
            // GetInput  -> Inherited from AbstractController
            // GetOutput -> Inherited from AbstractController

        protected:
            // TO-DO: How can we remove double memory allocation
            ParamsPid pid_params_;
            RobotState curr_state_;
        };

    } // namespace low

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_LOW_PID_HPP