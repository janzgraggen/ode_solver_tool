#include "Implicit.hh"
#include "../RootFinder/RootFinder.hh"
#include "../RootFinder/NewtonRaphson.hh"

using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;

bool Implicit::GetRhsIsLinear() const {
    return rhs_is_linear;
}

void Implicit::SetRhsIsLinear(bool is_linear) {
    rhs_is_linear = is_linear;
}

str Implicit::GetRootFinder() const {
    return root_finder;
}

LinearSystem Implicit::GetRhsSystem() const {
    return rhs_system;
}

void Implicit::SetRootFinder(str root_finder) {
    root_finder = root_finder;
}
void Implicit::SetRhsSystem(LinearSystem system) {
    rhs_system = system;
}

void Implicit::SetRightHandSide(const f_TYPE& f) {
    if (GetRhsIsLinear()) {
        // Set the right-hand side function

        // rhs is matMul + vec addition
        f_rhs = [this](const Eigen::VectorXd& y, double t) {
            return  this->GetRhsSystem().A * y + this->GetRhsSystem().b;
        };
        
    } else {
        // Set the right-hand side function
        this->f_rhs = f;
    }
}

Eigen::VectorXd Implicit::NonLinStep(const Eigen::VectorXd y, double t) {
    // Use RootFinder to solve the nonlinear system
        if (GetRootFinder() == "NewtonRaphson") {
            // Create a NewtonRaphson solver
            NewtonRaphson solver(makeFstep(y, t));

            // Set the initial guess
            solver.setInitialGuess(y);

            // Solve the nonlinear system
            return solver.Solve();
        } else {
            throw std::runtime_error("Invalid root finder method: " + GetRootFinder());
        }
}


Eigen::VectorXd Implicit::Step(const Eigen::VectorXd& y, double t) {
    if (GetRhsIsLinear()) {
        // Use LinStep to solve the linear system
        return LinStep(y, t);

    } else {
        // Use NonLinStep to solve the nonlinear system
        return NonLinStep(y, t);
    }
}

Eigen::VectorXd Implicit::SolveODE(std::ostream& stream) {
    Eigen::VectorXd y = GetInitialValue();  // Vector initialization.
    double t = GetInitialTime();

    // Output initial conditions.
    stream << "t: " << t << ", y: " << y.transpose() << std::endl;

    while (t <= GetFinalTime()) {
        // Call the specific step logic with vectors.
        y = Step(y, t);
        t += GetStepSize();
        // update F function based on new y,t



        stream << "t: " << t << ", y: " << y.transpose() << std::endl;
    }

    return y;
}

