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
#include "AbstractController.hpp"
#include "robot_controllers/Config.hpp"

namespace robot_controllers {
    void AbstractController::SetInput(const RobotState& input) { input_.desired_ = input; }

    void AbstractController::SetIOTypes(IOTypes input_type, IOTypes output_type)
    {
        // back-up old data
        RobotIO old_input = input_;
        RobotIO old_output = output_;

        // set new types
        input_ = RobotIO(input_type);
        input_.desired_ = old_input.desired_;

        output_ = RobotIO(output_type);
        output_.desired_ = old_output.desired_;
    }

    RobotIO AbstractController::GetInput() { return input_; }
    RobotIO AbstractController::GetOutput() { return output_; }

    void AbstractController::SetParams(const RobotParams& params) { params_ = params; }
    RobotParams AbstractController::GetParams() { return params_; }

    std::string AbstractController::pluginInterface()
    {
        return "RobotControllers.AbstractController/1.0";
    }

    std::vector<std::string> AbstractController::pluginSearchPaths()
    {
        return {std::string(ROBOT_CONTROLLERS_PLUGINS_DIR)};
    }
} // namespace robot_controllers