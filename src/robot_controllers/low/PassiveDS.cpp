#include <cassert>

#include "robot_controllers/low/PassiveDS.hpp"

namespace robot_controllers {
    namespace low {
        bool PassiveDS::Init()
        {
            passive_ds_params_.FromRobotParams(params_);
            Orthonormalize();
            AssertOrthonormalize();

            return true;
        }

        void PassiveDS::Update(const RobotState& state)
        {
            ComputeDamping();
            output_.desired_.force_ = -passive_ds_params_.damping_matrix_ * state.velocity_ + passive_ds_params_.eig_matrix_(0, 0) * input_.desired_.velocity_;
        }

        void PassiveDS::SetParams(unsigned int dim, const std::vector<double>& eigvals)
        {
            assert(eigvals.size() <= dim);

            params_.input_dim_ = dim;
            params_.output_dim_ = dim;

            // Fill the eigenvalue matrix
            passive_ds_params_.eig_matrix_ = Eigen::MatrixXd::Zero(dim, dim);

            for (size_t i = 0; i < eigvals.size(); i++)
                passive_ds_params_.eig_matrix_(i, i) = eigvals[i];

            for (size_t i = eigvals.size(); i < dim; i++)
                passive_ds_params_.eig_matrix_(i, i) = eigvals.back();
        }

        void PassiveDS::SetParams(const ParamsPassiveDS& params)
        {
            passive_ds_params_ = params;
            params_ = passive_ds_params_.ToRobotParams();
        }

        void PassiveDS::Orthonormalize()
        {
            assert(passive_ds_params_.basis_matrix_.rows() == passive_ds_params_.basis_matrix_.cols());
            unsigned int dim = passive_ds_params_.basis_matrix_.rows();
            passive_ds_params_.basis_matrix_.col(0).normalize();
            for (unsigned int i = 1; i < dim; i++) {
                for (unsigned int j = 0; j < i; j++)
                    passive_ds_params_.basis_matrix_.col(i) -= passive_ds_params_.basis_matrix_.col(j).dot(passive_ds_params_.basis_matrix_.col(i)) * passive_ds_params_.basis_matrix_.col(j);
                passive_ds_params_.basis_matrix_.col(i).normalize();
            }
        }

        void PassiveDS::AssertOrthonormalize()
        {
            uint dim = passive_ds_params_.basis_matrix_.cols();
            for (int i = 0; i < dim; ++i) {
                assert(abs(passive_ds_params_.basis_matrix_.col(i).norm() - 1.0) < FLOATEQUAL);
                for (int j = 0; j < i; ++j) {
                    assert(abs(passive_ds_params_.basis_matrix_.col(i).dot(passive_ds_params_.basis_matrix_.col(j))) < FLOATEQUAL);
                }
            }
        }

        void PassiveDS::ComputeOrthonormalBasis()
        {
            Eigen::VectorXd dir = input_.desired_.velocity_.normalized(); // or normalize
            assert(dir.rows() == passive_ds_params_.basis_matrix_.rows());
            passive_ds_params_.basis_matrix_.col(0) = dir;
            Orthonormalize();
        }

        void PassiveDS::ComputeDamping()
        {
            // only proceed of we have a minimum velocity norm!
            if (input_.desired_.velocity_.norm() > MINSPEED)
                ComputeOrthonormalBasis();
            // otherwise just use the last computed basis
            passive_ds_params_.damping_matrix_ = passive_ds_params_.basis_matrix_ * passive_ds_params_.eig_matrix_ * passive_ds_params_.basis_matrix_.transpose();
        }

    } // namespace low

} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(PassiveDSController, robot_controllers::low::PassiveDS, "RobotControllers.AbstractController/1.0")