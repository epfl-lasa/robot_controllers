#ifndef ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP
#define ROBOT_CONTROLLERS_HIGH_LINEAR_DS_HPP

#include <Eigen/Core>

#include <robot_controllers/AbstractController.hpp>

namespace robot_controllers {
    namespace high {
        struct InputLinearDS {
            Eigen::VectorXd current_position;
        };

        struct OutputLinearDS {
            Eigen::VectorXd desired_velocity_,
                desired_position_;
        };

        struct StateLinearDS {
            Eigen::MatrixXd A_;
            unsigned int dim_;
            double dt_;
        };

        class LinearDS : public robot_controllers::AbstractController<InputLinearDS, OutputLinearDS, StateLinearDS> {
        public:
            LinearDS();

            LinearDS(Eigen::MatrixXd A, double dt = 0.001);

            ~LinearDS();

            void SetInput(InputLinearDS input);

        protected:
            void Update() override;
        };
    } // namespace high
} // namespace robot_controllers

#endif