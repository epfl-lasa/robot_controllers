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
#ifndef ROBOT_CONTROLLERS_CASCADE_CONTROLLER_HPP
#define ROBOT_CONTROLLERS_CASCADE_CONTROLLER_HPP

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    class CascadeController : public AbstractController {
    public:
        explicit CascadeController(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin) {}

        CascadeController() : AbstractController() {}
        ~CascadeController() {}

        bool Init() override;
        void Update(const RobotState& state) override;

        void AddController(std::unique_ptr<AbstractController> controller);

        template <typename T, typename... Args>
        void AddController(Args... args)
        {
            auto ctrl_ptr = std::unique_ptr<T>(new T(std::forward<Args>(args)...));
            controllers_.emplace_back(std::move(ctrl_ptr));
        }

        AbstractController* GetController(unsigned int index);
        const AbstractController* GetController(unsigned int index) const;

        unsigned int NumControllers() const { return controllers_.size(); }

    protected:
        std::vector<std::unique_ptr<AbstractController>> controllers_;

        bool CheckConsistency();
    };

} // namespace robot_controllers

#endif