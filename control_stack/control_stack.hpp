#ifndef CONTROL_STACK_CONTROL_STACK_HPP
#define CONTROL_STACK_CONTROL_STACK_HPP

namespace control_stack {
    template <typename Input, typename Output, typename Params>
    class ControlStack {
    public:
        ControlStack() {}
        ~ControlStack() {}

        virtual bool Init() = 0;
        virtual void Update() = 0;

        Input GetInput() { return q_; }
        Output GetOutput() { return u_; }
        Params GetParams() { return state_; }

    protected:
        Input q_;
        Output u_;
        Params state_;
    };

} // namespace control_stack

#endif // CONTROL_STACK_CONTROL_STACK_HPP