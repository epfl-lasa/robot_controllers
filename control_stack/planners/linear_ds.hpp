#ifndef CONTROL_STACK_LINEAR_DS
#define CONTROL_STACK_LINEAR_DS

#include <control_stack/control_stack.hpp>

struct InputLinearDS
{
    Eigen::VectorXd current_position;
};

struct OutputLinearDS
{
    Eigen::VectorXd desired_velocity_,
                    desired_position_;
};

struct ParamsLinearDS
{
    Eigen::MatrixXd A_;
    unsigned int dim_;
    double dt_;
};

namespace control_stack
{
    namespace planners
    {
        class LinearDS : public control_stack::ControlStack<InputLinearDS,OutputLinearDS,ParamsLinearDS>
        {
        public:
            LinearDS();
            
            LinearDS(Eigen::MatrixXd A, double dt);
            
            ~LinearDS();

            void SetInput(InputLinearDS input);

        protected:
            void Update() override;
        };

    } // namespace planners

} // namespace control_stack

#endif // CONTROL_STACK_LINEAR_DS