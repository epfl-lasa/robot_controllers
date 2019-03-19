#ifndef ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP
#define ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP

#include <Eigen/Core>

#include <vector>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    namespace low {
        struct StatePassiveDS {
            Eigen::MatrixXd damping_matrix_,
                basis_matrix_,
                eig_matrix_;
            int state_dim_,
                num_eigval_;
        };

        struct InputPassiveDS {
            Eigen::VectorXd current_velocity_,
                desired_velocity_;
        };

        struct OutputPassiveDS {
            Eigen::VectorXd effort_;
        };

        class PassiveDS : public robot_controllers::AbstractController<InputPassiveDS, OutputPassiveDS, StatePassiveDS> {
        public:
            PassiveDS() {}
            ~PassiveDS() {}

            template <typename... Args>
            PassiveDS(unsigned int dim, Args... args)
            {
                // Set input state dimension and eigenvalues set
                state_.num_eigval_ = 0;
                state_.state_dim_ = dim;

                // Fill the eigenvalue matrix
                state_.eig_matrix_ = Eigen::MatrixXd::Zero(dim, dim);

                AddEigval(args...);

                for (size_t i = state_.num_eigval_; i < dim; i++)
                    state_.eig_matrix_(i, i) = state_.eig_matrix_(state_.num_eigval_ - 1, state_.num_eigval_ - 1);

                // Initialize the basis matrix
                state_.basis_matrix_ = Eigen::MatrixXd::Random(dim, dim);
                Orthonormalize();
                AssertOrthonormalize();

                // Initialize the damping matrix
                state_.damping_matrix_ = Eigen::MatrixXd::Zero(dim, dim);
            }

            bool Init() override;

            void SetInput(const Eigen::VectorXd& current, const Eigen::VectorXd& desired);

            void SetParams(unsigned int dim, const std::vector<double>& eigvals);

        protected:
            template <typename... Args>
            void AddEigval(double T, Args... args)
            {
                state_.eig_matrix_(state_.num_eigval_, state_.num_eigval_) = T;
                state_.num_eigval_++;
                AddEigval(args...);
            }

            void AddEigval(double T);

            void Update() override;

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