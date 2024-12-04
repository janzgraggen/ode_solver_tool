#ifndef __LU_SOLVE_HH__
#define __LU_SOLVE_HH__

#include "LinSysSolver.hh"

class LUSolve : public LinSysSolver {
public:
    LUSolve(); // Constructor
    ~LUSolve() override; // Destructor

    Eigen::VectorXd Solve() override; // Solve method
};

#endif // __LU_SOLVE_HH__