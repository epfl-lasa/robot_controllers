#ifndef ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP
#define ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP

#include <Eigen/Core>

#include <Corrade/Containers/EnumSet.h>

namespace robot_controllers {
    enum class IOType : unsigned int {
        Position = 1 << 0, // Contains position information
        Velocity = 1 << 1, // Contains velocity information
        Acceleration = 1 << 2, // Contains acceleration information
        Force = 1 << 3, // Contains force information
        All = Position | Velocity | Acceleration | Force // Contains everything
    }; // enum class IOTypes

    using IOTypes = Corrade::Containers::EnumSet<IOType>;

    CORRADE_ENUMSET_OPERATORS(IOTypes)

    struct RobotState {
        Eigen::VectorXd position_,
            velocity_,
            acceleration_,
            force_;
    };

    struct RobotIO {
        RobotIO(IOTypes type) : type_(type) {}

        RobotState desired_;

        // this should never change
        const IOTypes type_;
    }; // struct RobotIO

    class AbstractController {
    public:
        AbstractController(IOTypes input_type, IOTypes output_type) : input_(input_type), output_(output_type) {}
        virtual ~AbstractController() {}

        virtual bool Init() = 0;
        virtual void Update(const RobotState&) = 0;

        void SetInput(const RobotState& input) { input_.desired_ = input; }

        RobotIO GetInput() { return input_; }
        RobotIO GetOutput() { return output_; }

    protected:
        RobotIO input_;
        RobotIO output_;
    }; // class AbstractController

} // namespace robot_controllers

#endif