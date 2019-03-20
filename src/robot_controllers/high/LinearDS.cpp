#include <cassert>

#include "robot_controllers/high/LinearDS.hpp"

namespace robot_controllers {
    namespace high {
        LinearDS::LinearDS(Eigen::MatrixXd A, double dt) : AbstractController(IOType::Position, IOType::Velocity)
        {
            assert(A.rows() == A.cols());
            params_.A_ = A;
            params_.dim_ = A.rows();
            params_.dt_ = dt;
        }

        bool LinearDS::Init()
        {
            input_.desired_.position_ = Eigen::VectorXd::Zero(params_.dim_);
            return true;
        }

        void LinearDS::Update(const RobotState& state)
        {
            output_.desired_.velocity_ = params_.A_ * (input_.desired_.position_ - state.position_);
            output_.desired_.position_ = state.position_ + params_.dt_ * output_.desired_.velocity_;
        }

        void LinearDS::SetDesired(const Eigen::VectorXd& position)
        {
            input_.desired_.position_ = position;
        }
    } // namespace high
} // namespace robot_controllers