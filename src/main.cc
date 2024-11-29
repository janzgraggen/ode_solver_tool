#include <iostream>

#include "Ode/AdamsBashforth.hh"
#include "Ode/ForwardEuler.hh"
#include "Ode/RungeKutta.hh"

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

    // FORWARD EULER -------------------------------------------
    std::cout << "Forward Euler" << std::endl;
    FwdEuler FE_solver;

    // Set ODE parameters
    FE_solver.SetStepSize(0.01);
    FE_solver.SetTimeInterval(0.0, 1.0);

    FE_solver.SetInitialValue(initialValue);

    // Set the right-hand side function
    FE_solver.SetRightHandSide(ODEFunction);

    // Solve the ODE and output the results
    FE_solver.SolveODE(std::cout);

    std::cout << "\n\n\n" << std::endl;

    // RUNGE KUTTA -------------------------------------------
    std::cout << "Runge Kutta order 4" << std::endl;

    RungeKutta rk4(4);

    // Set ODE parameters
    rk4.SetStepSize(0.01);
    rk4.SetTimeInterval(0.0, 1.0);

    rk4.SetInitialValue(initialValue);

    // Set the right-hand side function
    rk4.SetRightHandSide(ODEFunction);

    // Solve the ODE and output the results
    rk4.SolveODE(std::cout);

    std::cout << "\n\n\n" << std::endl;

    // ADAMS BASHFORTH order 4 -------------------------------------------
    std::cout << "Adams Bashforth order 4" << std::endl;

    AdamsBashforth ab4(4);

    // Set ODE parameters
    ab4.SetStepSize(0.01);
    ab4.SetTimeInterval(0.0, 1.0);

    ab4.SetInitialValue(initialValue);

    // Set the right-hand side function
    ab4.SetRightHandSide(ODEFunction);

    // Solve the ODE and output the results
    ab4.SolveODE(std::cout);

    return 0;
}