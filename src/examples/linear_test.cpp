#include <iostream>
#include <vector>

#include <robot_controllers/CascadeController.hpp>
#include <robot_controllers/SumController.hpp>
#include <robot_controllers/high/LinearDS.hpp>
#include <robot_controllers/low/PassiveDS.hpp>

int main(int argc, char const* argv[])
{
    robot_controllers::low::PassiveDS pds;
    pds.SetParams(5, {1., 2.});

    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(5, 5);
    robot_controllers::high::LinearDS lds(A);

    robot_controllers::CascadeController cascade(robot_controllers::IOType::Position, robot_controllers::IOType::Force);
    cascade.AddController<robot_controllers::high::LinearDS>(A);
    cascade.AddController<robot_controllers::low::PassiveDS>(5, 1., 2.);

    std::cout << cascade.Init() << std::endl;

    robot_controllers::RobotState desired;
    desired.position_ = Eigen::VectorXd::Zero(5);
    cascade.SetInput(desired);

    robot_controllers::RobotState st;
    st.position_ = Eigen::VectorXd::Ones(5).array() * 0.1;
    st.velocity_ = Eigen::VectorXd::Zero(5);
    cascade.Update(st);

    std::cout << cascade.GetOutput().desired_.force_.transpose() << std::endl;

    robot_controllers::SumController sum(robot_controllers::IOType::Velocity, robot_controllers::IOType::Force);
    sum.AddController<robot_controllers::low::PassiveDS>(5, 1., 2.);
    sum.AddController<robot_controllers::low::PassiveDS>(5, 4., 4.);

    desired.velocity_ = Eigen::VectorXd::Ones(5);
    sum.SetInput(desired);

    sum.Update(st);

    std::cout << sum.GetOutput().desired_.force_.transpose() << std::endl;

    return 0;
}
