#include "linear_ds.hpp"
#include <assert.h>

namespace control_stack
{
    namespace planners
    {
        LinearDS::LinearDS() {}

        LinearDS::LinearDS(Eigen::MatrixXd A, double dt) 
        {
            assert( A.rows() == A.cols() );
            state_.A_ = A;
            state_.dim_ = A.rows();
            state_.dt_ = 0.001;
        }

        LinearDS::~LinearDS() {}

        void LinearDS::SetInput(InputLinearDS input)
        {
            q_ = input;
            Update();
        }

        void LinearDS::Update()
        {
            u_.desired_velocity_ = state_.A_*q_.current_position;
            u_.desired_position_ = q_.current_position + state_.dt_*u_.desired_velocity_;
        }
    }   // namespace planners

} // control_stack