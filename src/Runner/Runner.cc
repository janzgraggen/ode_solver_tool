#include "Runner.hh"
#include <iostream>
#include <memory>
#include <typeinfo>

#include "../Ode/ForwardEuler.hh"
#include "../Ode/RungeKutta.hh"
#include "../Ode/AdamsBashforth.hh"
#include "../Ode/BackwardEuler.hh"

#include "../Utils/LinSysStruct.hh"


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
            Solver = new RungeKutta();
        } else if (Rdr.getExplicitSettings().method == "AdamsBashforth") {
            Solver = new AdamsBashforth();
        } else {
            std::cout << "Invalid explicit solver method" << std::endl;
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else if (Rdr.getSolverType() == "Implicit") {

        if (Rdr.getImplicitSettings().method == "BackwardEuler") {
            Solver = new BackwardEuler();
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
        Solver->SetConfig(Rdr);
        Eigen::VectorXd Result = Solver->SolveODE();
        // Clean up
        delete Solver;  // Safely delete the dynamically allocated object
        Solver = nullptr;  // Best practice to avoid dangling pointers
        return Result; 
    }
}
