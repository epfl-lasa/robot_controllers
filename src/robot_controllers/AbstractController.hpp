#ifndef ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP
#define ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP

namespace robot_controllers {
    template <typename Input, typename Output, typename State>
    class AbstractController {
    public:
        AbstractController() {}
        ~AbstractController() {}

        virtual bool Init() = 0;
        virtual void Update() = 0;

        Input GetInput() { return input_; }
        Output GetOutput() { return output_; }
        State GetState() { return state_; }

    protected:
        Input input_;
        Output output_;
        State state_;
    };

} // namespace robot_controllers

#endif