#include "robot_controllers/low/Pid.hpp"

namespace robot_controllers
{
    namespace low
    {

        bool Pid::Init()
        {

        }

        void Pid::Update(const RobotState& state)
        {

        }

        void Pid::SetParams(const Eigen::MatrixXd& gain_matrix)
        {
            params_.gain_matrix_ = gain_matrix;
        }

        void Pid::SetDesired(const RobotState& state)
        {

        }

    }
}
