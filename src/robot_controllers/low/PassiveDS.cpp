//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
//|              Bernardo Fichera
//|    email:   costashatz@gmail.com
//|             bernardo.fichera@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of robot_controllers.
//|
//|    robot_controllers is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    robot_controllers is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#include <cassert>

#include "robot_controllers/low/PassiveDS.hpp"

namespace robot_controllers {
    namespace low {
        bool PassiveDS::Init()
        {
            assert(params_.input_dim_ == params_.output_dim_);
            // Initialize the basis matrix
            basis_matrix_ = Eigen::MatrixXd::Random(params_.input_dim_, params_.output_dim_);
            Orthonormalize();
            AssertOrthonormalize();

            // Initialize the damping matrix
            damping_matrix_ = Eigen::MatrixXd::Zero(params_.input_dim_, params_.output_dim_);

            SetParams(params_.input_dim_, params_.values_);

            return true;
        }

        void PassiveDS::Update(const RobotState& state)
        {
            ComputeDamping();
            output_.desired_.force_ = -damping_matrix_ * state.velocity_ + eig_matrix_(0, 0) * input_.desired_.velocity_;
        }

        void PassiveDS::SetParams(unsigned int dim, const std::vector<double>& eigvals)
        {
            assert(eigvals.size() <= dim && eigvals.size() > 0);

            params_.input_dim_ = dim;
            params_.output_dim_ = dim;

            // Fill the eigenvalue matrix
            eig_matrix_ = Eigen::MatrixXd::Zero(dim, dim);

            for (size_t i = 0; i < eigvals.size(); i++)
                eig_matrix_(i, i) = eigvals[i];

            for (size_t i = eigvals.size(); i < dim; i++)
                eig_matrix_(i, i) = eigvals.back();
        }

        void PassiveDS::Orthonormalize()
        {
            assert(basis_matrix_.rows() == basis_matrix_.cols());
            unsigned int dim = basis_matrix_.rows();
            basis_matrix_.col(0).normalize();
            for (unsigned int i = 1; i < dim; i++) {
                for (unsigned int j = 0; j < i; j++)
                    basis_matrix_.col(i) -= basis_matrix_.col(j).dot(basis_matrix_.col(i)) * basis_matrix_.col(j);
                basis_matrix_.col(i).normalize();
            }
        }

        void PassiveDS::AssertOrthonormalize()
        {
            uint dim = basis_matrix_.cols();
            for (int i = 0; i < dim; ++i) {
                assert(abs(basis_matrix_.col(i).norm() - 1.0) < FLOATEQUAL);
                for (int j = 0; j < i; ++j) {
                    assert(abs(basis_matrix_.col(i).dot(basis_matrix_.col(j))) < FLOATEQUAL);
                }
            }
        }

        void PassiveDS::ComputeOrthonormalBasis()
        {
            Eigen::VectorXd dir = input_.desired_.velocity_.normalized(); // or normalize
            assert(dir.rows() == basis_matrix_.rows());
            basis_matrix_.col(0) = dir;
            Orthonormalize();
        }

        void PassiveDS::ComputeDamping()
        {
            // only proceed of we have a minimum velocity norm!
            if (input_.desired_.velocity_.norm() > MINSPEED)
                ComputeOrthonormalBasis();
            // otherwise just use the last computed basis
            damping_matrix_ = basis_matrix_ * eig_matrix_ * basis_matrix_.transpose();
        }

    } // namespace low
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(PassiveDSController, robot_controllers::low::PassiveDS, "RobotControllers.AbstractController/1.0")
