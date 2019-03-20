#ifndef ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP
#define ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP

#include <Eigen/Core>

#include "robot_controllers/AbstractController.hpp"

namespace robot_controllers {
    namespace high {
        struct ParamsLinearDS {
            Eigen::MatrixXd A_;
            unsigned int dim_;
            double dt_;
        };

        class LinearDS : public AbstractController {
        public:
            LinearDS() : AbstractController(IOType::Position, IOType::Velocity) {}
            LinearDS(Eigen::MatrixXd A, double dt = 0.001);
            ~LinearDS() {}

            bool Init() override;
            void Update(const RobotState& state) override;

            void SetDesired(const Eigen::VectorXd& position);

        protected:
            ParamsLinearDS params_;
        };
    } // namespace high
} // namespace robot_controllers

#endif