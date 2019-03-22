#ifndef ROBOT_CONTROLLERS_LOW_PID_HPP
#define ROBOT_CONTROLLERS_LOW_PID_HPP

#include <robot_controllers/AbstractController.hpp>
#include <vector>

namespace robot_controllers {
    namespace low {
        struct ParamsPid {
            Eigen::MatrixXd p_matrix_,
                d_matrix_,
                i_matrix_;
            unsigned int input_dim_,
                output_dim_;
            double time_step_;
        };

        class Pid : public AbstractController {
        public:
            explicit Pid(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin)
            {
                input_.type_ = IOType::Position | IOType::Velocity;
                output_.type_ = IOType::Force;
            }

            Pid(const unsigned int input_dim, const unsigned int output_dim, const double time_step) : AbstractController(IOType::Position | IOType::Velocity, IOType::Force)
            {
                params_.input_dim_ = input_dim;
                params_.output_dim_ = output_dim;
                params_.time_step_ = time_step;
                Init();
            }

            ~Pid() {}

            bool Init() override;

            void Update(const RobotState& state) override;

            void SetParams(const Eigen::MatrixXd& p_matrix, const Eigen::MatrixXd& d_matrix, const Eigen::MatrixXd& i_matrix);

            // SetInput  -> Inherited from AbstractController
            // GetInput  -> Inherited from AbstractController
            // GetOutput -> Inherited from AbstractController

        protected:
            ParamsPid params_;
            RobotState curr_state_;
        };

    } // namespace low

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_LOW_PID_HPP