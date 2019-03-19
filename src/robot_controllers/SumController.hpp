#ifndef ROBOT_CONTROLLERS_SUM_CONTROLLER_HPP
#define ROBOT_CONTROLLERS_SUM_CONTROLLER_HPP

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    class SumController : public AbstractController {
    public:
        SumController(IOType input_type, IOType output_type) : AbstractController(input_type, output_type) {}
        ~SumController() {}

        bool Init() override;
        void Update(const RobotState& state) override;

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

        bool CheckConsistency() const;
    };

} // namespace robot_controllers

#endif