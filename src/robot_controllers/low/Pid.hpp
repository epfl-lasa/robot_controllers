#ifndef ROBOT_CONTROLLERS_LOW_PID_HPP
#define ROBOT_CONTROLLERS_LOW_PID_HPP

#include "robot_controllers/AbstractController.hpp"
#include <vector>

namespace robot_controllers {
    namespace low {
        struct ParamsPid {
            Eigen::MatrixXd gain_matrix_;
        };

        class Pid : public AbstractController {
        public:
            Pid() : AbstractController(IOType::Velocity, IOType::Force) {}

            ~Pid() {}

            bool Init() override;

            void Update(const RobotState& state) override;

            void SetParams(const Eigen::MatrixXd& gain_matrix);

            void SetDesired(const RobotState& state);

        protected:
            ParamsPid params_;
        };

    } // namespace low

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_LOW_PID_HPP