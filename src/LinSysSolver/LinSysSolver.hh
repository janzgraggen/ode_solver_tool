/**
 * @file LinSysSolver.hh
 * @brief Header file for the `LinSysSolver` class.
 *
 * This header defines the abstract base class `LinSysSolver` for solving linear
 * systems of equations \( Ax = b \). It provides methods to set and get the
 * coefficient matrix \( A \) and the right-hand side vector \( b \). Derived classes
 * must implement the `Solve` method to solve the system using a specific algorithm.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#ifndef __LIN_SYS_SOLVER_HH__
#define __LIN_SYS_SOLVER_HH__

#include <Eigen/Dense>
#include "../Logger/Logger.hh"

/**
 * @class LinSysSolver
 * @brief Abstract base class for solving linear systems \( Ax = b \).
 *
 * Provides a generic interface to set and retrieve the coefficient matrix \( A \)
 * and the right-hand side vector \( b \). Specific solving algorithms should be
 * implemented in derived classes.
 */
class LinSysSolver {

private:
    Eigen::MatrixXd A;  //!< Coefficient matrix \( A \) for the system \( Ax = b \).
    Eigen::VectorXd b;  //!< Right-hand side vector \( b \) for the system \( Ax = b \).

public:
    Logger* logger;  //!< Pointer to logger object for logging system messages.

    /**
     * @brief Default constructor for LinSysSolver.
     * @param logger_ Reference to the Logger object.
     */
    LinSysSolver(Logger& logger_);

    /**
     * @brief Virtual destructor for LinSysSolver.
     * @brief Ensures proper cleanup of derived classes.
     */
    virtual ~LinSysSolver();

    /**
     * @brief Sets the coefficient matrix \( A \).
     * @param A The coefficient matrix to be set.
     */
    void SetA(const Eigen::MatrixXd& A);

    /**
     * @brief Sets the right-hand side vector \( b \).
     * @param b The vector representing the right-hand side of the system.
     */
    void SetB(const Eigen::VectorXd& b);

    /**
     * @brief Retrieves the coefficient matrix \( A \).
     * @return A constant reference to the coefficient matrix \( A \).
     */
    const Eigen::MatrixXd& GetA() const;

    /**
     * @brief Retrieves the right-hand side vector \( b \).
     * @return A constant reference to the right-hand side vector \( b \).
     */
    const Eigen::VectorXd& GetB() const;

    /**
     * @brief Pure virtual method to solve the system \( Ax = b \).
     *
     * Derived classes must implement this method to solve the system using a
     * specific solving algorithm (e.g., Gaussian elimination, LU decomposition).
     *
     * @return Eigen::VectorXd The solution vector \( x \).
     */
    virtual Eigen::VectorXd Solve() = 0;
};

#endif // __LIN_SYS_SOLVER_HH__
