#include <robot_controllers/CascadeController.hpp>

namespace robot_controllers {
    bool CascadeController::Init()
    {
        for (auto& ctrl : controllers_)
            if (!ctrl->Init())
                return false;

        initialized_ = true;

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
        IOType t = input_.type_;
        for (auto& ctrl : controllers_) {
            if (ctrl->GetInput().type_ != t)
                return false;
            t = ctrl->GetOutput().type_;
        }

        return (controllers_.back()->GetOutput().type_ == output_.type_);
    }
} // namespace robot_controllers