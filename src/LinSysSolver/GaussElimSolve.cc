//
// Created by janzgraggen on 27/11/2024.
//

#include "GaussElimSolve.hh"

/**
 * @brief Solves the linear system of equations \( Ax = b \) using Gaussian elimination.
 *
 * This method utilizes Eigen's colPivHouseholderQr() decomposition to perform
 * the Gaussian elimination and solve for \( x \), where \( A \) is the coefficient
 * matrix and \( b \) is the right-hand side vector.
 *
 * @return Eigen::VectorXd The solution vector \( x \).
 */
Eigen::VectorXd GaussElimSolve::Solve() {
    // Retrieve the coefficient matrix (A) and the right-hand side vector (b)
    Eigen::MatrixXd A = this->GetA();
    Eigen::VectorXd b = this->GetB();

    // Solve the system using Eigen's QR decomposition
    return A.colPivHouseholderQr().solve(b);
}
