#include <Eigen/Dense>
#include <iostream>

#ifndef LIN_SYS_STRUCT_HH
#define LIN_SYS_STRUCT_HH

struct LinearSystem {
    Eigen::MatrixXd A; // The coefficient matrix
    Eigen::VectorXd b; // The right-hand side vector
    // default constructor: empty

    
    // Constructor to initialize with specific sizes
    LinearSystem(int rows, int cols)
        : A(Eigen::MatrixXd::Zero(rows, cols)), b(Eigen::VectorXd::Zero(rows)) {}

    // Constructor to directly initialize with given matrix and vector
    LinearSystem(const Eigen::MatrixXd& matrix, const Eigen::VectorXd& vector)
        : A(matrix), b(vector) {}
    
    // Default constructor
    LinearSystem() = default;

    // Method to print the system
    void print() const {
        std::cout << "Matrix A:\n" << A << "\n";
        std::cout << "Vector b:\n" << b << "\n";
    }
};

#endif // LIN_SYS_STRUCT_HH