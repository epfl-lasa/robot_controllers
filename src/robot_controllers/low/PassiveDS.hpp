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
#ifndef ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP
#define ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP

#include <robot_controllers/AbstractController.hpp>
#include <vector>

namespace robot_controllers {
    namespace low {
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

            void SetIOTypes(IOTypes input_type, IOTypes output_type) override {} // Do not allow changes in the IO types

            void Update(const RobotState& state) override;

            void SetParams(unsigned int dim, const std::vector<double>& eigvals);

            // SetInput  -> Inherited from AbstractController
            // GetInput  -> Inherited from AbstractController
            // GetOutput -> Inherited from AbstractController

        protected:
            Eigen::MatrixXd damping_matrix_,
                basis_matrix_,
                eig_matrix_;

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