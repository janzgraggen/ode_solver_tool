#ifndef __READER_HH__
#define __READER_HH__

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <optional>
#include "../Utils/LinSysStruct.hh"
#include "../Utils/SettingsStruct.hh"
#include "../Logger/Logger.hh"

using str = std::string;
using strList = std::vector<std::string>;
using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;

/**
 * @file Reader.hh
 * @brief Defines the `Reader` class for parsing ODE solver settings from a YAML configuration file.
 *
 * This header file defines the `Reader` class, which provides functionality to load and parse
 * configuration settings for ODE solvers from a YAML file. It supports retrieving general,
 * explicit, and implicit solver configurations and manages solver-specific parameters.
 */

/**
 * @class Reader
 * @brief A class for parsing configuration settings for ODE solvers from a YAML file.
 *
 * The `Reader` class reads and parses YAML configuration files to set up parameters for
 * ODE solvers, including solver types, function strings, and solver-specific settings.
 * It provides access to solver methods, coefficients, and initial conditions.
 */
class Reader {
public:
    Logger *logger; //!< The logger object for logging messages and solver activities.

    /**
     * @brief Constructs a `Reader` object and loads the specified YAML configuration file.
     * @param filename The path to the YAML configuration file.
     *
     * Loads and parses the specified YAML file containing ODE solver configuration settings.
     * Throws a `std::runtime_error` if the file cannot be opened or parsed.
     *
     * @throws std::runtime_error If the file cannot be opened or parsed.
     */
    Reader(Logger& logger,const str& filename);

    /**
     * @brief Retrieves the solver type specified in the configuration.
     * @return A string representing the solver type (e.g., "Implicit" or "Explicit").
     */
    str getSolverType() const;

    /**
     * @brief Retrieves the output filename specified in the configuration.
     * @return A string representing the output filename.
     */
    str getOutputFileName() const;

    /**
     * @brief Retrieves the dimension of the system specified in the configuration.
     * @return An integer representing the dimension of the system (number of state variables).
     */
    int getDim() const;

    /**
     * @brief Retrieves a list of function strings for the system from the configuration.
     *
     * Constructs a list of function strings corresponding to the dimension of the system.
     * These strings represent the mathematical functions defining the system's behavior.
     *
     * @return A `strList` containing the function strings.
     */
    strList getFunctionStringlist() const;

    /**
     * @brief Returns a callable function that evaluates the stored functions dynamically.
     *
     * Constructs and returns a callable `f_TYPE` that uses the `FunctionParser` to bind and evaluate
     * functions dynamically based on the configuration's state and time input.
     *
     * @return A callable `f_TYPE` lambda function that evaluates the system's functions given a state vector `y` and time `t`.
     */
    f_TYPE getFunction() const;

    /**
     * @brief Retrieves the verbosity level specified in the configuration.
     * @return An integer representing the verbosity level.
     */
    int getVerbosity() const;

    /**
     * @brief Sets the logger verbosity level based on the configuration.
     */
    void setLoggerVerbosity();

    /**
     * @brief Retrieves the general ODE solver settings from the configuration.
     * @return An `OdeSettings` structure containing the general solver parameters.
     */
    OdeSettings getOdeSettings() const;

    /**
     * @brief Retrieves the settings for explicit solvers from the configuration.
     * @return An `ExplicitSettings` structure containing the explicit solver parameters.
     */
    ExplicitSettings getExplicitSettings() const;

    /**
     * @brief Retrieves the settings for implicit solvers from the configuration.
     * @return An `ImplicitSettings` structure containing the implicit solver parameters.
     */
    ImplicitSettings getImplicitSettings() const;

private:
    YAML::Node config; //!< The YAML configuration object storing all configuration parameters.
};

#endif // __READER_HH__
