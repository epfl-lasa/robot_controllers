#ifndef ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP
#define ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP

#include <robot_controllers/AbstractController.hpp>
#include <vector>

namespace robot_controllers {
    namespace low {
        struct ParamsPassiveDS {
            Eigen::MatrixXd damping_matrix_,
                basis_matrix_,
                eig_matrix_;

            RobotParams ToRobotParams() const
            {
                assert(damping_matrix_.size() == basis_matrix_.size() && damping_matrix_.size() == eig_matrix_.size());
                RobotParams p;

                p.input_dim_ = damping_matrix_.rows();
                p.output_dim_ = damping_matrix_.cols();
                p.time_step_ = -1.; // we do not care about the time-step

                unsigned int size = damping_matrix_.size() + basis_matrix_.size() + eig_matrix_.size();

                if (size > 0) {
                    p.values_.resize(size);

                    Eigen::MatrixXd::Map(p.values_.data(), p.input_dim_, p.output_dim_) = damping_matrix_;
                    Eigen::MatrixXd::Map(p.values_.data() + damping_matrix_.size(), p.input_dim_, p.output_dim_) = basis_matrix_;
                    Eigen::MatrixXd::Map(p.values_.data() + damping_matrix_.size() + basis_matrix_.size(), p.input_dim_, p.output_dim_) = eig_matrix_;
                }

                return p;
            }

            void FromRobotParams(const RobotParams& p)
            {
                if (p.input_dim_ == 0 || p.output_dim_ == 0 || p.input_dim_ != p.output_dim_)
                    return;

                damping_matrix_.resize(p.input_dim_, p.output_dim_);
                basis_matrix_.resize(p.input_dim_, p.output_dim_);
                eig_matrix_.resize(p.input_dim_, p.output_dim_);

                damping_matrix_ = Eigen::MatrixXd::Map(p.values_.data(), p.input_dim_, p.output_dim_);
                basis_matrix_ = Eigen::MatrixXd::Map(p.values_.data() + damping_matrix_.size(), p.input_dim_, p.output_dim_);
                eig_matrix_ = Eigen::MatrixXd::Map(p.values_.data() + damping_matrix_.size() + basis_matrix_.size(), p.input_dim_, p.output_dim_);
            }
        };

        class PassiveDS : public AbstractController {
        public:
            explicit PassiveDS(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin)
            {
                input_ = RobotIO(IOType::Velocity);
                output_ = RobotIO(IOType::Force);
            }

            PassiveDS() : AbstractController(IOType::Velocity, IOType::Force) {}
            ~PassiveDS() {}

            bool Init() override;

            void Update(const RobotState& state) override;

            void SetParams(unsigned int dim, const std::vector<double>& eigvals);
            void SetParams(const ParamsPassiveDS& params);

            // SetInput  -> Inherited from AbstractController
            // GetInput  -> Inherited from AbstractController
            // GetOutput -> Inherited from AbstractController

        protected:
            ParamsPassiveDS passive_ds_params_;

            void Orthonormalize();

            void AssertOrthonormalize();

            void ComputeOrthonormalBasis();

            void ComputeDamping();

            // Missing function for set damping eigenvalue

            static constexpr double MINSPEED = 1e-6;
            static constexpr double FLOATEQUAL = 1e-6;
        };
    } // namespace low
} // namespace robot_controllers

#endif