#ifndef ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP
#define ROBOT_CONTROLLERS_LOW_PASSIVE_DS_HPP

#include <robot_controllers/AbstractController.hpp>
#include <vector>

namespace robot_controllers {
    namespace low {
        class PassiveDS : public AbstractController {
        public:
            explicit PassiveDS(Corrade::PluginManager::AbstractManager& manager, const std::string& plugin) : AbstractController(manager, plugin)
            {
                input_ = RobotIO(IOType::Velocity);
                output_ = RobotIO(IOType::Force);
            }

            PassiveDS() : AbstractController(IOType::Velocity, IOType::Force) {}
            ~PassiveDS() {}

            bool Init() override;

            void Update(const RobotState& state) override;

            void SetParams(unsigned int dim, const std::vector<double>& eigvals);

            // SetInput  -> Inherited from AbstractController
            // GetInput  -> Inherited from AbstractController
            // GetOutput -> Inherited from AbstractController

        protected:
            Eigen::MatrixXd damping_matrix_,
                basis_matrix_,
                eig_matrix_;

            void Orthonormalize();

            void AssertOrthonormalize();

            void ComputeOrthonormalBasis();

            void ComputeDamping();

            // Missing function for set damping eigenvalue

            static constexpr double MINSPEED = 1e-6;
            static constexpr double FLOATEQUAL = 1e-6;
        };
    } // namespace low
} // namespace robot_controllers

#endif