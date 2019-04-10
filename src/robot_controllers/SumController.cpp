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
#define add_result(name)                                          \
    {                                                             \
        if (result.name.size() == 0)                              \
            result.name = Eigen::VectorXd::Zero(out.name.size()); \
        result.name += out.name;                                  \
    }

        for (unsigned int i = 0; i < controllers_.size(); i++) {
            auto& ctrl = controllers_[i];
            ctrl->SetInput(input_.desired_);
            ctrl->Update(state);
            RobotState out = ctrl->GetOutput().desired_;

            IOTypes t = ctrl->GetOutput().GetType();
            if (t & IOType::Position)
                add_result(position_);
            if (t & IOType::Orientation)
                add_result(orientation_);
            if (t & IOType::Velocity)
                add_result(velocity_);
            if (t & IOType::AngularVelocity)
                add_result(angular_velocity_);
            if (t & IOType::Acceleration)
                add_result(acceleration_);
            if (t & IOType::AngularAcceleration)
                add_result(angular_acceleration_);
            if (t & IOType::Force)
                add_result(force_);
            if (t & IOType::Torque)
                add_result(torque_);
        }
#undef add_result
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
            return false;
        IOTypes in_type = controllers_.front()->GetInput().GetType();
        IOTypes out_type = controllers_.back()->GetOutput().GetType();

        for (auto& ctrl : controllers_) {
            in_type = in_type | ctrl->GetInput().GetType();
            out_type = out_type | ctrl->GetOutput().GetType();
        }

        input_ = RobotIO(in_type);
        output_ = RobotIO(out_type);

        return true;
    }
} // namespace robot_controllers

CORRADE_PLUGIN_REGISTER(SumController, robot_controllers::SumController, "RobotControllers.AbstractController/1.0")