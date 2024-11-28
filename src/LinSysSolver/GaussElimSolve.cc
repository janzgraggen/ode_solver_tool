
#include "GaussElimSolve.hh"

//add constructor and destructor implementation
GaussElimSolve::GaussElimSolve() {}
GaussElimSolve::~GaussElimSolve() {}

// Implement the Solve method 
Eigen::VectorXd GaussElimSolve::Solve() {
// Assuming A and b are members of LinSysSolver class
// A is the coefficient matrix and b is the right-hand side vector
Eigen::MatrixXd A = this->GetA();
Eigen::VectorXd b = this->GetB();
return A.colPivHouseholderQr().solve(b);

}