#ifndef CONTROL_STACK_CONTROL_STACK_HPP
#define CONTROL_STACK_CONTROL_STACK_HPP

#include <vector>
#include <Eigen/Dense>
#include "macros.hpp"
#include <iostream>

namespace control_stack {
    template <typename Input, typename Output, typename Params>
    class ControlStack
    {
    public:
        ControlStack() {}        
        virtual ~ControlStack() {}

        virtual void Init() {}
        virtual void Update() {}

        Input GetInputState() { return q_; }
        Output GetOutputState() { return u_; }
        Params GetParams() { return state_; }

    protected:
        Input q_;
        Output u_;
        Params state_;
    };
    
} // namespace control_stack


#endif // CONTROL_STACK_CONTROL_STACK_HPP