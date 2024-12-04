#include "BackwardEuler.hh"
#include "../LinSysSolver/GaussElimSolve.hh"
#include "../LinSysSolver/LUSolve.hh"

using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;

// ----
Eigen::VectorXd BackwardEuler::F(Eigen::VectorXd y1, Eigen::VectorXd y0,double t0){
    return y1 - y0 - GetStepSize() * GetRightHandSide()(y1, t0);
}

F_TYPE BackwardEuler::makeFstep(Eigen::VectorXd y0,double t0){
    // return function F(y1: variable, y0:set, t0:set) return a type: std::function<Eigen::VectorXd(const Eigen::VectorXd&)>
    return [this, y0, t0](const Eigen::VectorXd& y1) { return this->F(y1, y0, t0); };
}

void BackwardEuler::SetConfig(const Reader& Rdr) {
    SetRhsIsLinear(Rdr.getImplicitSettings().rhs_is_linear);
    SetLinearSystemSolver(Rdr.getImplicitSettings().linear_system_solver.value());

    if (GetRhsIsLinear()) {
        SetRhsSystem(Rdr.getImplicitSettings().rhs_system.value());
    } else {
        SetRootFinder(Rdr.getImplicitSettings().root_finder.value());
    }

    // After Set RHS System, call the SetGlobalConfig -> adds f accordingly
    SetGlobalConfig(Rdr);  // Call the base class method
}

Eigen::VectorXd BackwardEuler::LinStep(const Eigen::VectorXd y, double t) {
    // Solve the linear system

    // Create I + hA 
    Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(GetRhsSystem().A.rows(), GetRhsSystem().A.cols());
    Eigen::MatrixXd A = identity - GetStepSize() * GetRhsSystem().A;
    // Create b + hy
    Eigen::VectorXd b = GetStepSize() * GetRhsSystem().b + y; 
    if (GetLinearSystemSolver() == "GaussianElimination") {
        GaussElimSolve solver;
        solver.SetA(A);
        solver.SetB(b);
        return solver.Solve();

    // Solve the linear system
    return solver.Solve();
    } else if (GetLinearSystemSolver() == "LU") {
        LUSolve solver;
        solver.SetA(A);
        solver.SetB(b);
        return solver.Solve();

    }else {
        throw std::runtime_error("Invalid linear system solver: " + GetLinearSystemSolver());
    }

    
}



