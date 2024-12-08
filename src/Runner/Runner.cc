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
    Logger logger = Logger(Rdr.getVerbosity());
    OdeSolver* Solver = nullptr;
    if (Rdr.getSolverType() == "Explicit") {
        if (Rdr.getExplicitSettings().method == "ForwardEuler") {
            Solver = new FwdEuler(logger);
            logger.info("Forward Euler solver instance created");
        } else if (Rdr.getExplicitSettings().method == "RungeKutta") {
            Solver = new RungeKutta(logger);
            logger.info("Runge-Kutta solver instance created");
        } else if (Rdr.getExplicitSettings().method == "AdamsBashforth") {
            Solver = new AdamsBashforth(logger);
            logger.info("Adams-Bashforth solver instance created");
        } else {
            logger.info("Invalid explicit solver method");
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else if (Rdr.getSolverType() == "Implicit") {

        if (Rdr.getImplicitSettings().method == "BackwardEuler") {
            Solver = new BackwardEuler(logger);
        } else {
            logger.info("Invalid implicit solver method");
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else {
        logger.error("Invalid solver type");
        return Eigen::VectorXd();  // Return empty vector for failure
    }
    
    if (Solver == nullptr) {
        logger.error("Solver instance not created");
        delete Solver;
        return Eigen::VectorXd();  // Return empty vector for failure
    } else {
        Solver->SetConfig(Rdr);
        logger.info("Solver configured successfully");
        Eigen::VectorXd Result = Solver->SolveODE();
        logger.info("Solving completed successfully");
        // Clean up
        delete Solver;  // Safely delete the dynamically allocated object
        Solver = nullptr;  // Best practice to avoid dangling pointers
        return Result; 
        logger.info("Solver instance deleted for clean up");
    }
}
