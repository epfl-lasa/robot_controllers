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
            double time_step_;

            RobotParams ToRobotParams() const;
            void FromRobotParams(const RobotParams& p);
        };

        class Pid : public AbstractController {
        public:
            explicit Pid(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin)
            {
                input_ = RobotIO(IOType::Position | IOType::Velocity);
                output_ = RobotIO(IOType::Force);
            }

            Pid(const unsigned int input_dim, const unsigned int output_dim, const double time_step) : AbstractController(IOType::Position | IOType::Velocity, IOType::Force)
            {
                params_.input_dim_ = input_dim;
                params_.output_dim_ = output_dim;
                params_.time_step_ = time_step;
            }

            ~Pid() {}

            bool Init() override;

            void Update(const RobotState& state) override;

            void SetParams(const ParamsPid& params);

        protected:
            ParamsPid pid_params_;
            RobotState state_;
            Eigen::VectorXd intergral_error_;
            bool has_orientation_, has_position_;
            unsigned int dim_;
        };

    } // namespace low

} // namespace robot_controllers

#endif // ROBOT_CONTROLLERS_LOW_PID_HPP