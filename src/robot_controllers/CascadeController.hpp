#ifndef ROBOT_CONTROLLERS_CASCADE_CONTROLLER_HPP
#define ROBOT_CONTROLLERS_CASCADE_CONTROLLER_HPP

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    class CascadeController : public AbstractController {
    public:
        CascadeController(IOType input_type, IOType output_type) : AbstractController(input_type, output_type), initialized_(false) {}
        ~CascadeController() {}

        bool Init() override;
        void Update(const RobotState& state) override;

        template <typename T, typename... Args>
        void AddController(Args... args)
        {
            auto ctrl_ptr = std::unique_ptr<T>(new T(std::forward<Args>(args)...));
            controllers_.emplace_back(std::move(ctrl_ptr));

            // // if already initialized, check for consistency
            // if (initialized_)
            //     CheckConsistency();
        }

        AbstractController* GetController(unsigned int index);
        const AbstractController* GetController(unsigned int index) const;

        unsigned int NumControllers() const { return controllers_.size(); }

    protected:
        std::vector<std::unique_ptr<AbstractController>> controllers_;
        bool initialized_;

        bool CheckConsistency() const;
    };

} // namespace robot_controllers

#endif