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

    bool CascadeController::CheckConsistency() const
    {
        IOTypes t = IOType::All;
        for (auto& ctrl : controllers_) {
            if (!(ctrl->GetInput().type_ & t))
                return false;
            t = ctrl->GetOutput().type_;
        }

        if (controllers_.back()->GetOutput().type_ & output_.type_)
            return true;

        return false;
    }
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(CascadeController, robot_controllers::CascadeController, "RobotControllers.AbstractController/1.0")