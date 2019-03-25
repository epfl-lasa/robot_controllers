#include <robot_controllers/SumController.hpp>

namespace robot_controllers {
    bool SumController::Init()
    {
        for (auto& ctrl : controllers_)
            if (!ctrl->Init())
                return false;

        return CheckConsistency();
    }

    void SumController::Update(const RobotState& state)
    {
        RobotState result;
        for (unsigned int i = 0; i < controllers_.size(); i++) {
            auto& ctrl = controllers_[i];
            ctrl->SetInput(input_.desired_);
            ctrl->Update(state);
            RobotState out = ctrl->GetOutput().desired_;
            if (i == 0)
                result = out;
            else {
                IOTypes t = ctrl->GetOutput().GetType();
                if (t & IOType::Position)
                    result.position_ += out.position_;
                if (t & IOType::Velocity)
                    result.velocity_ += out.velocity_;
                if (t & IOType::Acceleration)
                    result.acceleration_ += out.acceleration_;
                if (t & IOType::Force)
                    result.force_ += out.force_;
            }
        }
        output_.desired_ = result;
    }

    void SumController::AddController(std::unique_ptr<AbstractController> controller)
    {
        controllers_.emplace_back(std::move(controller));
    }

    AbstractController* SumController::GetController(unsigned int index)
    {
        assert(index < controllers_.size());
        return controllers_[index].get();
    }

    const AbstractController* SumController::GetController(unsigned int index) const
    {
        assert(index < controllers_.size());
        return controllers_[index].get();
    }

    bool SumController::CheckConsistency()
    {
        if (controllers_.size() == 0)
            return true;
        input_ = RobotIO(controllers_.front()->GetInput().GetType());
        output_ = RobotIO(controllers_.back()->GetOutput().GetType());

        auto& c = controllers_[0];
        IOTypes it = c->GetInput().GetType();
        IOTypes ot = c->GetOutput().GetType();
        for (auto& ctrl : controllers_) {
            if (!(ctrl->GetInput().GetType() & it))
                return false;
            if (!(ctrl->GetOutput().GetType() & ot))
                return false;
        }

        return true;
    }
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(SumController, robot_controllers::SumController, "RobotControllers.AbstractController/1.0")