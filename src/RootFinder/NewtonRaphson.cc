//
// Created by janzgraggen on 27/11/2024.
//
#include "NewtonRaphson.hh"
#include "GaussElimSolve.hh"
#include <iostream>

// Default constructor
NewtonRaphson::NewtonRaphson(Eigen::VectorXd (*F_in)(Eigen::VectorXd))
    : RootFinder(F_in), dx(1e-6) {}

// Parameterized constructor
NewtonRaphson::NewtonRaphson(Eigen::VectorXd (*F_in)(Eigen::VectorXd), double tol, double dx, int maxIter)
    : RootFinder(F_in), dx(dx) { // Use provided dx
    setTolerance(tol);
    setMaxIterations(maxIter);
}

// Destructor
NewtonRaphson::~NewtonRaphson() {}

// Setter
void NewtonRaphson::setDx(double dx) {
    this->dx = dx;
}

// Getter
double NewtonRaphson::getDx() const {
    return dx;
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

    GaussElimSolve solver;

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

    return x;
}