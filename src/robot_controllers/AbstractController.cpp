#include "AbstractController.hpp"
#include "robot_controllers/Config.hpp"

namespace robot_controllers {
    void AbstractController::SetInput(const RobotState& input) { input_.desired_ = input; }

    RobotIO AbstractController::GetInput() { return input_; }
    RobotIO AbstractController::GetOutput() { return output_; }

    std::string AbstractController::pluginInterface()
    {
        return "RobotControllers.AbstractController/1.0";
    }

    std::vector<std::string> AbstractController::pluginSearchPaths()
    {
        return {std::string(ROBOT_CONTROLLERS_PLUGINS_DIR)};
    }
} // namespace robot_controllers