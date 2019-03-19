#include <cassert>

#include "LinearDS.hpp"

namespace robot_controllers {
    namespace high {
        LinearDS::LinearDS() {}

        LinearDS::LinearDS(Eigen::MatrixXd A, double dt)
        {
            assert(A.rows() == A.cols());
            state_.A_ = A;
            state_.dim_ = A.rows();
            state_.dt_ = dt;
        }

        LinearDS::~LinearDS() {}

        void LinearDS::SetInput(InputLinearDS input)
        {
            input_ = input;
            Update();
        }

        void LinearDS::Update()
        {
            output_.desired_velocity_ = state_.A_ * input_.current_position_;
            output_.desired_position_ = input_.current_position_ + state_.dt_ * output_.desired_velocity_;
        }
    } // namespace high
} // namespace robot_controllers