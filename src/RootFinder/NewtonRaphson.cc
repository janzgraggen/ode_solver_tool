//
// Created by janzgraggen on 27/11/2024.
//
#include "NewtonRaphson.hh"
#include "../LinSysSolver/GaussElimSolve.hh"
#include "../LinSysSolver/LUSolve.hh"
#include <iostream>

using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;
using str = std::string;
// Default constructor
NewtonRaphson::NewtonRaphson(Logger& logger_ ,F_TYPE F_in)
    : RootFinder(logger_,F_in), dx(1e-6) {  // Default step size dx set to 1e-6
}

// Parameterized constructor
NewtonRaphson::NewtonRaphson(Logger& logger_,F_TYPE F_in, double tol, double dx, int maxIter)
    : RootFinder(logger_,F_in), dx(dx){ // Use provided dx
    setTolerance(tol);
    setMaxIterations(maxIter);
}

// Destructor
NewtonRaphson::~NewtonRaphson() {}

// Setter
void NewtonRaphson::setDx(double dx) {
    this->dx = dx;
}

void NewtonRaphson::SetLinearSystemSolver(str linear_system_solver_in) {
    linear_system_solver = linear_system_solver_in;
}
// Getter
double NewtonRaphson::getDx() const {
    return dx;
}

str NewtonRaphson::GetLinearSystemSolver() const {
    return linear_system_solver;
}

// Numerical Jacobian computation
Eigen::MatrixXd NewtonRaphson::NumericalJacobian(Eigen::VectorXd& x) {
    double dx = getDx();
    int n = x.size();
    Eigen::MatrixXd J(callF(x).size(), n);

    Eigen::VectorXd xPlus = x, xMinus = x;

    for (int j = 0; j < n; j++) {
        xPlus(j) += dx;
        xMinus(j) -= dx;

        Eigen::VectorXd fPlus = callF(xPlus);
        Eigen::VectorXd fMinus = callF(xMinus);

        for (int i = 0; i < fPlus.size(); i++) {
            J(i, j) = (fPlus(i) - fMinus(i)) / (2 * dx);
        }

        xPlus(j) = x(j);
        xMinus(j) = x(j);
    }

    return J;
}

// Solve function
Eigen::VectorXd NewtonRaphson::Solve() {
    Eigen::VectorXd x = getInitialGuess();
    Eigen::VectorXd Fx = callF(x); 
    if (GetLinearSystemSolver() == "GaussianElimination") {
        GaussElimSolve solver(logger);

        while (Fx.norm() > getTolerance() && getIterationCount() < getMaxIterations()) {
            Eigen::MatrixXd J = NumericalJacobian(x);

            try {
                solver.SetA(J);
                solver.SetB(-Fx);

                Eigen::VectorXd delta = solver.Solve();
                x += delta;

                Fx = callF(x);
                setIterationCount(getIterationCount() + 1);
            
            } catch (std::exception& e) {
                std::cerr << "Error during solving: " << e.what() << std::endl;
                break;
            }
        }
        

        if (getIterationCount() == getMaxIterations()) {
            std::cerr << "Newton-Raphson did not converge within the maximum number of iterations." << std::endl;
        }

    } else if (GetLinearSystemSolver() == "LU") {
        LUSolve solver(logger);

        while (Fx.norm() > getTolerance() && getIterationCount() < getMaxIterations()) {
            Eigen::MatrixXd J = NumericalJacobian(x);

            try {
                solver.SetA(J);
                solver.SetB(-Fx);

                Eigen::VectorXd delta = solver.Solve();
                x += delta;

                Fx = callF(x);
                setIterationCount(getIterationCount() + 1);
            
            } catch (std::exception& e) {
                std::cerr << "Error during solving: " << e.what() << std::endl;
                break;
            }
        }
        

        if (getIterationCount() == getMaxIterations()) {
            std::cerr << "Newton-Raphson did not converge within the maximum number of iterations." << std::endl;
        }
    } else {
        std::cerr << "Invalid linear system solver" << std::endl;
    }

    return x;
}