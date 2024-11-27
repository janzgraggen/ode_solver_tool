#include <iostream>
#include <Eigen/Dense>
#include "ForwardEuler.hpp"
#include "RungeKutta.hpp"

// Vectorized ODE function: dy/dt = -y + t
Eigen::VectorXd ODEFunction(const Eigen::VectorXd& y, double t) {
    Eigen::VectorXd f(y.size());
    for (int i = 0; i < y.size(); ++i) {
        f(i) = y(i);  // Example: dy/dt = -y + t
    }
    return f;
}

int main() {

    Eigen::VectorXd initialValue(3);  // Vector of size 3
    initialValue << 0.0, 1.0, 0.0;  // Initial values for y(0)

    // Create the Forward Euler solver
    FwdEuler FE_solver;

    // Set ODE parameters
    FE_solver.SetStepSize(0.01);
    FE_solver.SetTimeInterval(0.0, 1.0);

    FE_solver.SetInitialValue(initialValue);

    // Set the right-hand side function
    FE_solver.SetRightHandSide(ODEFunction);

    // Solve the ODE and output the results
    FE_solver.SolveODE(std::cout);

    RungeKutta rk4(4);

    // Set ODE parameters
    rk4.SetStepSize(0.01);
    rk4.SetTimeInterval(0.0, 1.0);

    rk4.SetInitialValue(initialValue);

    // Set the right-hand side function
    rk4.SetRightHandSide(ODEFunction);

    // Solve the ODE and output the results
    rk4.SolveODE(std::cout);

    return 0;
}