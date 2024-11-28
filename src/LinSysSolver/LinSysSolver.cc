#include <LinSysSolver.hh>

// Constructor  
LinSysSolver::LinSysSolver() {}

// Destructor
LinSysSolver::~LinSysSolver() {}

// Setters
void LinSysSolver::SetA(const Eigen::MatrixXd& A) {
    this->A = A;
}

void LinSysSolver::SetB(const Eigen::VectorXd& b) {
    this->b = b;
}

// Getters

const Eigen::MatrixXd& LinSysSolver::GetA() const {
    return A;
}

const Eigen::VectorXd& LinSysSolver::GetB() const {
    return b;
}