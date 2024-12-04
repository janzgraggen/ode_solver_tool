//
// Created by natha on 25/11/2024.
//

#include "ForwardEuler.hh"

FwdEuler::FwdEuler() {}
FwdEuler::~FwdEuler() {}

void FwdEuler::SetConfig(const Reader& Rdr) {
    SetGlobalConfig(Rdr);  // Call the base class method
}

Eigen::VectorXd FwdEuler::Step(const Eigen::VectorXd& y, double t) {
    Eigen::VectorXd f = GetRightHandSide()(y, t);  // Call the vectorized RHS function
    return y + GetStepSize() * f;  // Forward Euler step update
}