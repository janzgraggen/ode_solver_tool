/**
 * @file LinSysStruct.hh
 * @brief Defines the `LinearSystem` struct representing a linear system of equations.
 *
 * This header file contains the definition of the `LinearSystem` struct, which models a system of
 * linear equations in the form **Ax = b**. It includes:
 * - A coefficient matrix **A** (represented by `Eigen::MatrixXd`).
 * - A right-hand side vector **b** (represented by `Eigen::VectorXd`).
 *
 * The `LinearSystem` struct provides constructors for:
 * - Initializing with matrix dimensions (default sizes set to zero).
 * - Initializing with a given matrix and vector.
 *
 * It also includes a method to print the system's matrix and vector for debugging or logging purposes.
 *
 * @author: janzgraggen
 * @date: 27/11/2024
 */

#ifndef LIN_SYS_STRUCT_HH
#define LIN_SYS_STRUCT_HH

#include <Eigen/Dense>
#include <iostream>

/**
 * @struct LinearSystem
 * @brief Represents a linear system of equations in the form A * x = b.
 *
 * This struct contains a coefficient matrix `A` and a right-hand side vector `b`.
 * It includes constructors to initialize the matrix and vector with different methods.
 */
struct LinearSystem {
    /** @brief The coefficient matrix (A) in the system of equations. */
    Eigen::MatrixXd A;

    /** @brief The right-hand side vector (b) in the system of equations. */
    Eigen::VectorXd b;

    /**
     * @brief Constructor to initialize the system with specific matrix sizes.
     *
     * Initializes the matrix `A` and vector `b` with zeros of given dimensions.
     *
     * @param rows Number of rows in the matrix.
     * @param cols Number of columns in the matrix.
     */
    LinearSystem(int rows, int cols)
        : A(Eigen::MatrixXd::Zero(rows, cols)), b(Eigen::VectorXd::Zero(rows)) {}

    /**
     * @brief Constructor to directly initialize the system with a given matrix and vector.
     *
     * Sets the matrix `A` and the right-hand side vector `b` to the provided values.
     *
     * @param matrix The coefficient matrix `A`.
     * @param vector The right-hand side vector `b`.
     */
    LinearSystem(const Eigen::MatrixXd& matrix, const Eigen::VectorXd& vector)
        : A(matrix), b(vector) {}

    /** @brief Default constructor. Initializes an empty system. */
    LinearSystem() = default;

    /**
     * @brief Prints the coefficient matrix `A` and the right-hand side vector `b`.
     *
     * Outputs the matrix and vector to the console for debugging or inspection purposes.
     */
    void print() const {
        std::cout << "Matrix A:\n" << A << "\n";
        std::cout << "Vector b:\n" << b << "\n";
    }
};

#endif // LIN_SYS_STRUCT_HH