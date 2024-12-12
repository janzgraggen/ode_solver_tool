/**
 * @file Runner.cc
 * @brief Implementation of the Runner class responsible for configuring and running ODE solvers.
 *
 * This file contains the definition of the `Runner` class, which initializes and configures
 * appropriate ODE solvers based on the provided configuration file. The class dynamically
 * allocates solver instances (such as Forward Euler, Runge-Kutta, Adams-Bashforth, and Backward Euler)
 * according to the solver type and method specified in the configuration. It also handles logging
 * information and ensures proper memory management by cleaning up allocated solver instances.
 *
 * The `Runner` class interacts with the `Logger` class and configuration reader `Rdr` to determine
 * the solver settings and log relevant messages about solver creation, configuration, and execution.
 *
 * @author: janzgraggen
 * @date: 27/11/2024
 */

#include "Runner.hh"

#include "../Ode/ForwardEuler.hh"
#include "../Ode/RungeKutta.hh"
#include "../Ode/AdamsBashforth.hh"
#include "../Ode/BackwardEuler.hh"
#include "../Logger/Logger.hh"


using str = std::string; //< Alias for string type

/**
 * @brief Constructor for the Runner class.
 *
 * Initializes a `Runner` instance with a given configuration file and logger.
 *
 * @param logger_ Reference to the Logger instance for logging messages.
 * @param config_file The path to the configuration file containing ODE solver parameters.
 */
Runner::Runner(Logger& logger_, const std::string& config_file) :
    config_file(config_file), Rdr(logger_, config_file) , logger(&logger_){}

/**
 * @brief Destructor for the Runner class.
 *
 * Cleans up resources when an instance of the Runner class is destroyed.
 */
Runner::~Runner() {}

/**
 * @brief Main method to run the ODE solver.
 *
 * This method:
 * - Reads the solver type from the configuration.
 * - Creates a corresponding ODE solver instance dynamically.
 * - Configures the solver according to the provided settings.
 * - Solves the ODE and returns the result as an Eigen::VectorXd.
 *
 * @return Eigen::VectorXd The result of solving the ODE.
 */
Eigen::VectorXd Runner::run() {
    Rdr.setLoggerVerbosity();
    OdeSolver* Solver = nullptr;

    // Determine solver type and method from the configuration
    str SolverType = Rdr.getSolverType();
    if (SolverType == "Explicit") {
        str methodName = Rdr.getExplicitSettings().method; 
        if (methodName == "ForwardEuler") {
            Solver = new FwdEuler(*logger);
            logger->info("{in Runner::run()} Forward Euler solver instance created");
        } else if (methodName == "RungeKutta") {
            Solver = new RungeKutta(*logger);
            logger->info("{in Runner::run()} Runge-Kutta solver instance created");
        } else if (methodName == "AdamsBashforth") {
            Solver = new AdamsBashforth(*logger);
            logger->info("{in Runner::run()} Adams-Bashforth solver instance created");
        } else {
            logger->error("{in Runner::run()} Invalid explicit solver method: Rdr.getExplicitSettings().method returned:" + methodName);
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else if (SolverType == "Implicit") {
        str methodName = Rdr.getImplicitSettings().method; 
        if (methodName == "BackwardEuler") {
            Solver = new BackwardEuler(*logger);
            logger->info("{in Runner::run()} Backward Euler solver instance created");
        } else {
            logger->error("{in Runner::run()} Invalid implicit solver method: Rdr.getImplicitSettings().method returned:" + methodName);
            return Eigen::VectorXd();  // Return empty vector for failure
        }
    } else {
        logger->error("{in Runner::run()} Invalid solver type: Rdr.getSolverType() returned:" + SolverType);
        return Eigen::VectorXd();  // Return empty vector for failure
    }

    // Check if solver instance creation was successful
    if (Solver == nullptr) {
        logger->error("{in Runner::run()} Solver instance creation failed");
        delete Solver;
        return Eigen::VectorXd();  // Return empty vector for failure
    } else {
        Solver->setConfig(Rdr);
        logger->info("{in Runner::run()} Solver configured successfully");

        Eigen::VectorXd Result = Solver->solveOde();
        logger->info("{in Runner::run()} Solving completed successfully");

        // Clean up dynamically allocated memory
        delete Solver;  // Safely delete the solver instance
        logger->info("{in Runner::run()} Solver instance deleted for clean up");
        Solver = nullptr;  // Best practice to avoid dangling pointers

        return Result;
    }
}
