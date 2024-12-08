//
// Created by janzgraggen on 27/11/2024.
//
#ifndef __NEWTON_RAPHSON__HH__
#define __NEWTON_RAPHSON__HH__

#include "RootFinder.hh"
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;
using str = std::string;

class NewtonRaphson : public RootFinder {
private:
    double dx; // Step size for numerical differentiation
    str linear_system_solver; // linear system solver to use for implicit methods


public:
    // Constructors (need because additional dx argument to initialize)
    // Default constructor using std::function for flexibility
    NewtonRaphson(Logger& logger_,F_TYPE F_in);

    // Parameterized constructor using std::function for flexibility
    NewtonRaphson(Logger& logger_,F_TYPE F_in, double tol, double dx, int maxIter);

    // Destructor
    virtual ~NewtonRaphson();

    // Setter and getter for dx
    void setDx(double dx);
    void SetLinearSystemSolver(str solver);
    double getDx() const;
    str GetLinearSystemSolver() const;

    // Numerical Jacobian computation
    Eigen::MatrixXd NumericalJacobian(Eigen::VectorXd& x);

    // Overridden Solve function
    Eigen::VectorXd Solve() override;
};

#endif // __NEWTON_RAPHSON__HH__
