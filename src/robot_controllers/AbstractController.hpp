#ifndef ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP
#define ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP

namespace robot_controllers {
    enum class IOType : unsigned int {
        Position = 1 << 0, // Contains position information
        Velocity = 1 << 1, // Contains velocity information
        Acceleration = 1 << 2, // Contains acceleration information
        Force = 1 << 3, // Contains force information
        All = Position | Velocity | Acceleration | Force // Contains everything
    };

    struct RobotState {
        Eigen::VectorXd position_,
            velocity_,
            acceleration_,
            force_;
    };

    struct RobotIO {
        RobotIO(IOType type) : type_(type) {}

        RobotState desired_;

        // this should never change
        const IOType type_;
    };

    class AbstractController {
    public:
        AbstractController(IOType input_type, IOType output_type) : input_(input_type), output_(output_type) {}
        ~AbstractController() {}

        virtual bool Init() = 0;
        virtual void Update(const RobotState&) = 0;

        void SetInput(const RobotState& input) { input_.desired_ = input; }

        RobotIO GetInput() { return input_; }
        RobotIO GetOutput() { return output_; }

    protected:
        RobotIO input_;
        RobotIO output_;
    };

} // namespace robot_controllers

#endif