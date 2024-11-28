//
// Created by janzgraggen on 27/11/2024.
//
#ifndef __NEWTON_RAPHSON__HH__
#define __NEWTON_RAPHSON__HH__

#include "RootFinder.hh"

class NewtonRaphson : public RootFinder {
private:
    double dx; // Step size for numerical differentiation

public:
    // Constructors
    NewtonRaphson(Eigen::VectorXd (*F_in)(Eigen::VectorXd));
    NewtonRaphson(Eigen::VectorXd (*F_in)(Eigen::VectorXd), double tol, double dx, int maxIter);

    // Destructor
    virtual ~NewtonRaphson();

    // Setter and getter for dx
    void setDx(double dx);
    double getDx() const;

    // Numerical Jacobian computation
    Eigen::MatrixXd NumericalJacobian(Eigen::VectorXd& x);

    // Overridden Solve function
    Eigen::VectorXd Solve() override;
};

#endif // __NEWTON_RAPHSON__HH__
