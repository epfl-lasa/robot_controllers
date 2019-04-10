#ifndef ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP
#define ROBOT_CONTROLLERS_ABSTRACT_CONTROLLER_HPP

#include <Eigen/Core>

#include <string>
#include <vector>

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/PluginManager/AbstractManagingPlugin.h>

namespace robot_controllers {
    enum class IOType : unsigned int {
        Position = 1 << 0, // Contains position information
        Orientation = 1 << 1, // Contains orientation information
        Velocity = 1 << 2, // Contains velocity information
        AngularVelocity = 1 << 3, // Contains angular velocity information
        Acceleration = 1 << 4, // Contains acceleration information
        AngularAcceleration = 1 << 5, // Contains angular acceleration information
        Force = 1 << 6, // Contains force information
        Torque = 1 << 7, // Contains torque information
        All = Position | Orientation | Velocity | AngularVelocity | Acceleration | AngularAcceleration | Force | Torque // Contains everything
    }; // enum class IOTypes

    using IOTypes = Corrade::Containers::EnumSet<IOType>;
    CORRADE_ENUMSET_OPERATORS(IOTypes)

    struct RobotState {
        Eigen::VectorXd position_,
            velocity_,
            acceleration_,
            force_;
        Eigen::VectorXd orientation_,
            angular_velocity_,
            angular_acceleration_,
            torque_;
    };

    struct RobotIO {
    public:
        RobotIO() {}
        RobotIO(IOTypes type) : type_(type) {}

        IOTypes GetType() const { return type_; }

        RobotState desired_;

    private:
        // this should never change
        IOTypes type_;
    }; // struct RobotIO

    struct RobotParams {
        unsigned int input_dim_, output_dim_;
        double time_step_;
        std::vector<double> values_;
    };

    class AbstractController : public Corrade::PluginManager::AbstractManagingPlugin<AbstractController> {
    public:
        explicit AbstractController(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : Corrade::PluginManager::AbstractManagingPlugin<AbstractController>{manager, plugin} {}

        AbstractController() {}
        AbstractController(IOTypes input_type, IOTypes output_type) : input_(input_type), output_(output_type) {}
        virtual ~AbstractController() {}

        // Init should be called after SetParams
        virtual bool Init() = 0;
        virtual void Update(const RobotState&) = 0;

        void SetInput(const RobotState& input);
        virtual void SetIOTypes(IOTypes input_type, IOTypes output_type);
        RobotIO GetInput();
        RobotIO GetOutput();

        void SetParams(const RobotParams& params);
        RobotParams GetParams();

        // Corrade Plugin Methods
        static std::string pluginInterface();
        static std::vector<std::string> pluginSearchPaths();

    protected:
        RobotIO input_;
        RobotIO output_;
        RobotParams params_;
    }; // class AbstractController

} // namespace robot_controllers

#endif