//
// Created by natha on 25/11/2024.
//

#include "ForwardEuler.hh"

FwdEuler::FwdEuler() {}
FwdEuler::~FwdEuler() {}

Eigen::VectorXd FwdEuler::Step(const Eigen::VectorXd& y, double t) {
    Eigen::VectorXd f = GetRightHandSide()(y, t);  // Call the vectorized RHS function
    return y + GetStepSize() * f;  // Forward Euler step update
}