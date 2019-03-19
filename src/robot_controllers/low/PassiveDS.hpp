#ifndef ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP
#define ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP

#include <Eigen/Core>

#include <vector>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    namespace low {
        struct ParamsPassiveDS {
            Eigen::MatrixXd damping_matrix_,
                basis_matrix_,
                eig_matrix_;
            int params_dim_,
                num_eigval_;
        };

        class PassiveDS : public AbstractController {
        public:
            PassiveDS() : AbstractController(IOType::Velocity, IOType::Force) {}
            ~PassiveDS() {}

            template <typename... Args>
            PassiveDS(unsigned int dim, Args... args) : AbstractController(IOType::Velocity, IOType::Force)
            {
                // Set input state dimension and eigenvalues set
                params_.num_eigval_ = 0;
                params_.params_dim_ = dim;

                // Fill the eigenvalue matrix
                params_.eig_matrix_ = Eigen::MatrixXd::Zero(dim, dim);

                AddEigval(args...);

                for (size_t i = params_.num_eigval_; i < dim; i++)
                    params_.eig_matrix_(i, i) = params_.eig_matrix_(params_.num_eigval_ - 1, params_.num_eigval_ - 1);

                // Initialize the basis matrix
                params_.basis_matrix_ = Eigen::MatrixXd::Random(dim, dim);
                Orthonormalize();
                AssertOrthonormalize();

                // Initialize the damping matrix
                params_.damping_matrix_ = Eigen::MatrixXd::Zero(dim, dim);
            }

            bool Init() override;

            void SetParams(unsigned int dim, const std::vector<double>& eigvals);

            void Update(const RobotState& state) override;

            void SetDesired(const Eigen::VectorXd& velocity);

        protected:
            ParamsPassiveDS params_;

            template <typename... Args>
            void AddEigval(double T, Args... args)
            {
                params_.eig_matrix_(params_.num_eigval_, params_.num_eigval_) = T;
                params_.num_eigval_++;
                AddEigval(args...);
            }

            void AddEigval(double T);

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