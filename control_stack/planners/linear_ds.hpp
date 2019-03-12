#ifndef CONTROL_STACK_LINEAR_DS
#define CONTROL_STACK_LINEAR_DS

#include "control_stack.hpp"

struct ParamsLinearDS
{

};

namespace control_stack
{
    namespace planners
    {
        class LinearDS : public control_stack::ControlStack<Eigen::VectorXd,Eigen::VectorXd, ParamsLinearDS>
        {
        public:
            LinearDS();
            ~LinearDS();
        };

    } // namespace planners

} // namespace control_stack

#endif // CONTROL_STACK_LINEAR_DS