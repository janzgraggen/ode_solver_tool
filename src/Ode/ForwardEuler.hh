//
// Created by natha on 25/11/2024.
//

#ifndef FORWARD_EULER_HH
#define FORWARD_EULER_HH

#include "Explicit.hh"

class FwdEuler : public Explicit {
public:
    FwdEuler();
    ~FwdEuler();

    void SetConfig(const Reader& Rdr) override;
    
    Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;  // Vectorized Step
};

#endif // FORWARD_EULER_HH
