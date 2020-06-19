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
#include <iostream>
#include <vector>

#include <robot_controllers/CascadeController.hpp>
#include <robot_controllers/SumController.hpp>
#include <robot_controllers/high/LinearDS.hpp>
#include <robot_controllers/low/PassiveDS.hpp>
#include <robot_controllers/low/Pid.hpp>

int main(int argc, char const* argv[])
{
    Corrade::PluginManager::Manager<robot_controllers::AbstractController> manager;

    if (!(manager.load("PassiveDSController") & Corrade::PluginManager::LoadState::Loaded)) {
        std::cout << "BA" << std::endl;
    }

    Corrade::Containers::Pointer<robot_controllers::AbstractController> ctrl = manager.instantiate("PassiveDSController");
    auto c = static_cast<robot_controllers::low::PassiveDS*>(ctrl.get());
    // c->SetParams(5, {1., 2.});
    // robot_controllers::low::PassiveDS pds;
    // pds.SetParams(5, {1., 2.});

    // Eigen::MatrixXd A = Eigen::MatrixXd::Identity(5, 5);
    // robot_controllers::high::LinearDS lds(A);

    // robot_controllers::CascadeController cascade(robot_controllers::IOType::Position, robot_controllers::IOType::Force);
    // cascade.AddController<robot_controllers::high::LinearDS>(A);
    // cascade.AddController<robot_controllers::low::PassiveDS>(5, 1., 2.);

    // std::cout << cascade.Init() << std::endl;

    // robot_controllers::RobotState desired;
    // desired.position_ = Eigen::VectorXd::Zero(5);
    // cascade.SetInput(desired);

    // robot_controllers::RobotState st;
    // st.position_ = Eigen::VectorXd::Ones(5).array() * 0.1;
    // st.velocity_ = Eigen::VectorXd::Zero(5);
    // cascade.Update(st);

    // std::cout << cascade.GetOutput().desired_.force_.transpose() << std::endl;

    // robot_controllers::SumController sum(robot_controllers::IOType::Velocity, robot_controllers::IOType::Force);
    // sum.AddController<robot_controllers::low::PassiveDS>(5, 1., 2.);
    // sum.AddController<robot_controllers::low::PassiveDS>(5, 4., 4.);

    // desired.velocity_ = Eigen::VectorXd::Ones(5);
    // sum.SetInput(desired);

    // sum.Update(st);

    // std::cout << sum.GetOutput().desired_.force_.transpose() << std::endl;

    return 0;
}
