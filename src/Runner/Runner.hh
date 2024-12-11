/**
* @file Runner.hh
 * @brief Defines the `Runner` class responsible for orchestrating the ODE solvers based on a configuration file.
 *
 * This header file contains the definition of the `Runner` class, which:
 * - Reads a configuration file to determine solver types and methods.
 * - Uses the `Reader` class to handle YAML parsing for extracting solver settings.
 * - Provides a `run` method to dynamically create and execute the appropriate ODE solver.
 *
 * The `Runner` class encapsulates the interactions with the configuration and solver classes,
 * ensuring the selected ODE solvers are correctly configured and executed while maintaining
 * proper memory management and logging.
 *
 * @author: janzgraggen
 * @date: 27/11/2024
 */

#ifndef RUNNER_HH
#define RUNNER_HH

#include "../Reader/Reader.hh"  // Header for the Reader class, responsible for reading YAML configuration
#include "../Ode/OdeSolver.hh"  // Header for the OdeSolver class, the base class for ODE solvers
#include <string>

/**
 * @class Runner
 * @brief A class to configure and run ODE solvers based on a configuration file.
 *
 * The `Runner` class reads a configuration file to determine the solver type and method.
 * It dynamically creates the corresponding ODE solver instance, configures it, and executes
 * the solver to solve the ODE. The class interacts with the `Reader` class to parse the
 * configuration file and the `Logger` class for logging messages.
 */
class Runner {
private:
    /** @brief Configuration file containing solver parameters and methods. */
    std::string config_file;

    /** @brief Reader instance responsible for parsing the configuration YAML file. */
    Reader Rdr;

public:
    Logger* logger;  // Logger instance for logging messages
    /**
     * @brief Constructor for the Runner class.
     *
     * Initializes the Runner instance with a given configuration file.
     *
     * @param config_file The path to the configuration file containing ODE solver parameters.
     */
    Runner(Logger& logger_ , const std::string& config_file);

    /**
     * @brief Destructor for the Runner class.
     *
     * Cleans up any resources when the Runner instance is destroyed.
     */
    ~Runner();

    /**
     * @brief Main method to run the ODE solver.
     *
     * Reads the solver type and method from the configuration file, dynamically creates
     * the corresponding solver instance, configures it, and then executes the solver.
     *
     * @return Eigen::VectorXd The computed result of solving the ODE.
     */
    Eigen::VectorXd run();
};

#endif // RUNNER_HH
