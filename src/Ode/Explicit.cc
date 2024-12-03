//
// Created by natha on 25/11/2024.
//

#include "Explicit.hh"

Eigen::VectorXd Explicit::SolveODE(std::ostream& stream) {
    Eigen::VectorXd y = GetInitialValue();  // Vector initialization.
    double t = GetInitialTime();

    // Output initial conditions.
    stream << "t: " << t << ", y: " << y.transpose() << std::endl;

    while (t <= GetFinalTime()) {
        // Call the specific step logic with vectors.
        y = Step(y, t);
        t += GetStepSize();
        stream << "t: " << t << ", y: " << y.transpose() << std::endl;
    }

    return y;
}