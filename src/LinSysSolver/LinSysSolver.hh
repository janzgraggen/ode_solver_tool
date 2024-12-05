//
// Created by janzgraggen on 27/11/2024.
//
#ifndef __LIN_SYS_SOLVER_HH__
#define __LIN_SYS_SOLVER_HH__

#include <Eigen/Dense>

/**
 * @class LinSysSolver
 * @brief Abstract base class for solving linear systems of equations \( Ax = b \).
 *
 * This class defines a common interface for linear system solvers. It provides
 * methods to set and retrieve the coefficient matrix \( A \) and the right-hand side
 * vector \( b \). Derived classes must implement the `Solve` method to solve the system.
 */
class LinSysSolver {
  
private:
    Eigen::MatrixXd A;  //!< The coefficient matrix \( A \) in the system \( Ax = b \).
    Eigen::VectorXd b;  //!< The right-hand side vector \( b \) in the system \( Ax = b \).

public:
    /**
     * @brief Default constructor for LinSysSolver.
     */
    LinSysSolver();

    /**
     * @brief Virtual destructor for LinSysSolver.
     *
     * Ensures proper cleanup of derived classes.
     */
    virtual ~LinSysSolver();

    /**
     * @brief Sets the coefficient matrix \( A \) for the linear system.
     * @param A The coefficient matrix to be set.
     */
    void SetA(const Eigen::MatrixXd& A);

    /**
     * @brief Sets the right-hand side vector \( b \) for the linear system.
     * @param b The right-hand side vector to be set.
     */
    void SetB(const Eigen::VectorXd& b);

    /**
     * @brief Retrieves the coefficient matrix \( A \).
     * @return A constant reference to the coefficient matrix.
     */
    const Eigen::MatrixXd& GetA() const;

    /**
     * @brief Retrieves the right-hand side vector \( b \).
     * @return A constant reference to the right-hand side vector.
     */
    const Eigen::VectorXd& GetB() const;

    /**
     * @brief Pure virtual method to solve the linear system \( Ax = b \).
     *
     * This method must be implemented by derived classes to solve the system
     * using a specific algorithm.
     *
     * @return Eigen::VectorXd The solution vector \( x \).
     */
    virtual Eigen::VectorXd Solve() = 0;
};
#endif // __LIN_SYS_SOLVER_HH__
