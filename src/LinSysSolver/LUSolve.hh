#ifndef __LU_SOLVE_HH__
#define __LU_SOLVE_HH__

#include "LinSysSolver.hh"

/**
 * @class LUSolve
 * @brief A class to solve linear systems \( Ax = b \) using LU decomposition.
 *
 * This class extends the `LinSysSolver` base class and provides an implementation
 * of the `Solve` method. It utilizes Eigen's `FullPivLU` for decomposition to
 * handle both invertible and singular matrices.
 */
class LUSolve : public LinSysSolver {
public:
    /**
     * @brief Default constructor for the LUSolve class.
     */
    LUSolve();

    /**
     * @brief Destructor for the LUSolve class.
     */
    ~LUSolve() override;

    /**
     * @brief Solves the linear system \( Ax = b \) using LU decomposition.
     *
     * This method overrides the `Solve` method in the `LinSysSolver` base class.
     * It retrieves the coefficient matrix \( A \) and the right-hand side vector \( b \),
     * performs LU decomposition using Eigen's `FullPivLU`, and solves the system.
     *
     * @return Eigen::VectorXd The solution vector \( x \).
     * @throws std::runtime_error if the coefficient matrix \( A \) is singular.
     */
    Eigen::VectorXd Solve() override;
};

#endif // __LU_SOLVE_HH__
