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

using str = std::string;

// Constructor
Runner::Runner(const std::string& config_file) :
    config_file(config_file), Rdr(config_file) {}

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
    Logger logger = Logger(Rdr.getVerbosity());
    OdeSolver* Solver = nullptr;

    // Determine solver type and method from the configuration
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

    // Check if solver instance creation was successful
    if (Solver == nullptr) {
        logger.error("Solver instance not created");
        delete Solver;
        return Eigen::VectorXd();  // Return empty vector for failure
    } else {
        Solver->SetConfig(Rdr);
        logger.info("Solver configured successfully");

        Eigen::VectorXd Result = Solver->SolveODE();
        logger.info("Solving completed successfully");

        // Clean up dynamically allocated memory
        delete Solver;  // Safely delete the solver instance
        logger.info("Solver instance deleted for clean up");
        Solver = nullptr;  // Best practice to avoid dangling pointers

        return Result;
    }
}
