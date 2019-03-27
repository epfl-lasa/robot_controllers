#ifndef ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP
#define ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP

#include <Eigen/Core>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    namespace high {
        struct ParamsLinearDS {
            Eigen::MatrixXd A_;
            double time_step_;

            RobotParams ToRobotParams() const
            {
                RobotParams p;

                p.input_dim_ = A_.rows();
                p.output_dim_ = A_.cols();
                p.time_step_ = time_step_;

                unsigned int size = A_.size();

                if (size > 0) {
                    p.values_.resize(size);

                    Eigen::MatrixXd::Map(p.values_.data(), p.input_dim_, p.output_dim_) = A_;
                }

                return p;
            }

            void FromRobotParams(const RobotParams& p)
            {
                if (p.input_dim_ == 0 || p.output_dim_ == 0)
                    return;

                time_step_ = p.time_step_;

                A_ = Eigen::MatrixXd::Zero(p.input_dim_, p.output_dim_);

                unsigned int size = p.values_.size();
                // if only one element
                if (size == 1) {
                    A_.diagonal() = Eigen::VectorXd::Constant(p.input_dim_, p.values_[0]);
                }
                else if (size == p.input_dim_) { // diagonal elements
                    A_.diagonal() = Eigen::VectorXd::Map(p.values_.data(), p.input_dim_);
                }
                else { // full matrix
                    A_ = Eigen::MatrixXd::Map(p.values_.data(), p.input_dim_, p.output_dim_);
                }
            }
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
            void Update(const RobotState& state) override;

            void SetParams(const ParamsLinearDS& params);

        protected:
            ParamsLinearDS linear_ds_params_;
        };
    } // namespace high
} // namespace robot_controllers

#endif