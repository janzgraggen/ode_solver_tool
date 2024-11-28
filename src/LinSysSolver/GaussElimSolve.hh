//
// Created by janzgraggen on 27/11/2024.
//
#ifndef __GAUSSELIMSOLVE_HH__
#define __GAUSSELIMSOLVE_HH__

#include <LinSysSolver.hh>


class GaussElimSolve : public LinSysSolver {

/* ------------------------------------------------------------------------ */
/* Members                                                                  */
/* ------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------ */
/* Methods                                                                  */
/* ------------------------------------------------------------------------ */

// Override solve method from LinSysSolver
public:     
    Eigen::VectorXd Solve() override; // Declaration of the Solve method
};
#endif // __GAUSSELIMSOLVE_HH__