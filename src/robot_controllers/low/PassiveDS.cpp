#include <cassert>

#include "robot_controllers/low/PassiveDS.hpp"

namespace robot_controllers {
    namespace low {
        bool PassiveDS::Init()
        {
            // Initialize the basis matrix
            params_.basis_matrix_ = Eigen::MatrixXd::Random(params_.params_dim_, params_.params_dim_);
            Orthonormalize();
            AssertOrthonormalize();

            // Initialize the damping matrix
            params_.damping_matrix_ = Eigen::MatrixXd::Zero(params_.params_dim_, params_.params_dim_);

            return true;
        }

        void PassiveDS::Update(const RobotState& state)
        {
            ComputeDamping();
            output_.desired_.force_ = -params_.damping_matrix_ * state.velocity_ + params_.eig_matrix_(0, 0) * input_.desired_.velocity_;
        }

        void PassiveDS::SetParams(unsigned int dim, const std::vector<double>& eigvals)
        {
            // Set input state dimension and eigenvalues set
            params_.num_eigval_ = eigvals.size();
            params_.params_dim_ = dim;

            // Fill the eigenvalue matrix
            params_.eig_matrix_ = Eigen::MatrixXd::Zero(dim, dim);

            for (size_t i = 0; i < eigvals.size(); i++)
                params_.eig_matrix_(i, i) = eigvals[i];

            for (size_t i = params_.num_eigval_; i < dim; i++)
                params_.eig_matrix_(i, i) = eigvals.back();

            Init();
        }

        void PassiveDS::AddEigval(double T)
        {
            params_.eig_matrix_(params_.num_eigval_, params_.num_eigval_) = T;
            params_.num_eigval_++;
        }

        void PassiveDS::Orthonormalize()
        {
            assert(params_.basis_matrix_.rows() == params_.basis_matrix_.cols());
            unsigned int dim = params_.basis_matrix_.rows();
            params_.basis_matrix_.col(0).normalize();
            for (unsigned int i = 1; i < dim; i++) {
                for (unsigned int j = 0; j < i; j++)
                    params_.basis_matrix_.col(i) -= params_.basis_matrix_.col(j).dot(params_.basis_matrix_.col(i)) * params_.basis_matrix_.col(j);
                params_.basis_matrix_.col(i).normalize();
            }
        }

        void PassiveDS::AssertOrthonormalize()
        {
            uint dim = params_.basis_matrix_.cols();
            for (int i = 0; i < dim; ++i) {
                assert(abs(params_.basis_matrix_.col(i).norm() - 1.0) < FLOATEQUAL);
                for (int j = 0; j < i; ++j) {
                    assert(abs(params_.basis_matrix_.col(i).dot(params_.basis_matrix_.col(j))) < FLOATEQUAL);
                }
            }
        }

        void PassiveDS::ComputeOrthonormalBasis()
        {
            Eigen::VectorXd dir = input_.desired_.velocity_.normalized(); // or normalize
            assert(dir.rows() == params_.basis_matrix_.rows());
            params_.basis_matrix_.col(0) = dir;
            Orthonormalize();
        }

        void PassiveDS::ComputeDamping()
        {
            // only proceed of we have a minimum velocity norm!
            if (input_.desired_.velocity_.norm() > MINSPEED)
                ComputeOrthonormalBasis();
            // otherwise just use the last computed basis
            params_.damping_matrix_ = params_.basis_matrix_ * params_.eig_matrix_ * params_.basis_matrix_.transpose();
        }

    } // namespace low

} // namespace robot_controllers