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
#include <robot_controllers/CascadeController.hpp>

namespace robot_controllers {
    bool CascadeController::Init()
    {
        for (auto& ctrl : controllers_)
            if (!ctrl->Init())
                return false;

        return CheckConsistency();
    }

    void CascadeController::Update(const RobotState& state)
    {
        RobotState curr = input_.desired_;
        for (auto& ctrl : controllers_) {
            ctrl->SetInput(curr);
            ctrl->Update(state);
            curr = ctrl->GetOutput().desired_;
        }
        output_.desired_ = curr;
    }

    void CascadeController::AddController(std::unique_ptr<AbstractController> controller)
    {
        controllers_.emplace_back(std::move(controller));
    }

    AbstractController* CascadeController::GetController(unsigned int index)
    {
        assert(index < controllers_.size());
        return controllers_[index].get();
    }

    const AbstractController* CascadeController::GetController(unsigned int index) const
    {
        assert(index < controllers_.size());
        return controllers_[index].get();
    }

    bool CascadeController::CheckConsistency()
    {
        if (controllers_.size() == 0)
            return false;
        input_ = RobotIO(controllers_.front()->GetInput().GetType());
        output_ = RobotIO(controllers_.back()->GetOutput().GetType());

        IOTypes t = IOType::All;
        for (auto& ctrl : controllers_) {
            if (!(ctrl->GetInput().GetType() & t))
                return false;
            t = ctrl->GetOutput().GetType();
        }

        if (controllers_.back()->GetOutput().GetType() & output_.GetType())
            return true;

        return false;
    }
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(CascadeController, robot_controllers::CascadeController, "RobotControllers.AbstractController/1.0")