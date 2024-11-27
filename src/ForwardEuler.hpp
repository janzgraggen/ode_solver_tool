//
// Created by natha on 25/11/2024.
//

#ifndef FORWARD_EULER_HPP
#define FORWARD_EULER_HPP

#include "Explicit.hpp"

class FwdEuler : public Explicit {
public:
    FwdEuler();
    ~FwdEuler();

    Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;  // Vectorized Step
};

#endif // FORWARD_EULER_HPP
