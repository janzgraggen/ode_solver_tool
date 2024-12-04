#include "Runner.hh"
#include <iostream>
#include <memory>
#include <typeinfo>

#include "../Ode/ForwardEuler.hh"
#include "../Ode/RungeKutta.hh"
#include "../Ode/AdamsBashforth.hh"
#include "../Ode/BackwardEuler.hh"

#include "../LinSysStruct.hh"

#include "../config/Config.hh"

using str = std::string;

//constructor
Runner::Runner(const std::string& config_file) : 
    config_file(config_file), Rdr(config_file) {}


//destructor
Runner::~Runner() {}

//run method

Eigen::VectorXd Runner::run() {
    OdeSolver* Solver = nullptr;
    if (Rdr.getSolverType() == "Explicit") {
        if (Rdr.getExplicitSettings().method == "ForwardEuler") {
            Solver = new FwdEuler();

        } else if (Rdr.getExplicitSettings().method == "RungeKutta") {
            // If order is provided
            if (Rdr.getExplicitSettings().RungeKutta_order.has_value()) {
                Solver = new RungeKutta(Rdr.getExplicitSettings().RungeKutta_order.value());
                // If coefficients a, b, c are provided
            } else if (Rdr.getExplicitSettings().RungeKutta_coefficients_a.has_value() &&
                       Rdr.getExplicitSettings().RungeKutta_coefficients_b.has_value() &&
                       Rdr.getExplicitSettings().RungeKutta_coefficients_c.has_value()) {
                Solver = new RungeKutta(
                    Rdr.getExplicitSettings().RungeKutta_coefficients_a.value(),
                    Rdr.getExplicitSettings().RungeKutta_coefficients_b.value(),
                    Rdr.getExplicitSettings().RungeKutta_coefficients_c.value());
            } else {
                std::cout << "Invalid RungeKutta settings" << std::endl;
                return Eigen::VectorXd();  // Return empty vector for failure
            }
        } else if (Rdr.getExplicitSettings().method == "AdamsBashforth") {
            if (Rdr.getExplicitSettings().AdamsBashforth_max_order.has_value()){
                Solver = new AdamsBashforth(
                    Rdr.getExplicitSettings().AdamsBashforth_max_order.value());
            } else if (Rdr.getExplicitSettings().AdamsBashforth_coefficients_vector.has_value()) {
                Solver = new AdamsBashforth(Rdr.getExplicitSettings().AdamsBashforth_coefficients_vector.value());
            } else {
                std::cout << "Invalid AdamsBashforth settings" << std::endl;
                return {};  // Return empty vector for failure
            }

        } else {
            std::cout << "Invalid explicit solver method" << std::endl;
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else if (Rdr.getSolverType() == "Implicit") {

        if (Rdr.getImplicitSettings().method == "BackwardEuler") {
            Solver = new BackwardEuler();
            dynamic_cast<BackwardEuler*>(Solver)->SetRhsIsLinear(Rdr.getImplicitSettings().rhs_is_linear);
            dynamic_cast<BackwardEuler*>(Solver)->SetLinearSystemSolver(Rdr.getImplicitSettings().linear_system_solver.value());
            if (dynamic_cast<BackwardEuler*>(Solver)->GetRhsIsLinear()) {
                dynamic_cast<BackwardEuler*>(Solver)->SetRhsSystem(Rdr.getImplicitSettings().rhs_system.value());
            } else {
                dynamic_cast<BackwardEuler*>(Solver)->SetRootFinder(Rdr.getImplicitSettings().root_finder.value());
            }
        } else {
            std::cout << "Invalid implicit solver method" << std::endl;
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else {
        std::cout << "Invalid solver type" << std::endl;
        return Eigen::VectorXd();  // Return empty vector for failure
    }
    
    if (Solver == nullptr) {
        std::cout << "Solver not created" << std::endl;
        delete Solver;
        return Eigen::VectorXd();  // Return empty vector for failure
    } else {

        Solver->SetStepSize(Rdr.getOdeSettings().step_size);
        Solver->SetTimeInterval(Rdr.getOdeSettings().initial_time, Rdr.getOdeSettings().final_time);
        Solver->SetInitialValue(Rdr.getOdeSettings().initial_value);
        Solver->SetRightHandSide(f); 
        Eigen::VectorXd Result  = Solver->SolveODE(std::cout);
        
        // Clean up
        delete Solver;  // Safely delete the dynamically allocated object
        Solver = nullptr;  // Best practice to avoid dangling pointers

        return Result;
    
}
}
