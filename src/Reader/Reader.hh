#ifndef __READER_HH__
#define __READER_HH__

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <optional>
#include "../Utils/LinSysStruct.hh"

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

    /**
     * @brief Constructs a `Reader` object and loads the specified YAML configuration file.
     * @param filename The path to the YAML configuration file.
     *
     * Loads and parses the specified YAML file containing ODE solver configuration settings.
     * Throws a `std::runtime_error` if the file cannot be opened or parsed.
     *
     * @throws std::runtime_error If the file cannot be opened or parsed.
     */
    Reader(const str& filename);

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
     * @struct OdeSettings
     * @brief Holds general settings for ODE solvers.
     *
     * Contains basic time integration parameters for the ODE solver, such as step size,
     * initial time, final time, and initial state vector.
     */
    struct OdeSettings {
        double step_size;          //!< The step size for time integration.
        double initial_time;       //!< The initial time of the simulation.
        double final_time;         //!< The final time of the simulation.
        Eigen::VectorXd initial_value; //!< The initial state vector.
    };

    /**
     * @struct ExplicitSettings
     * @brief Holds settings for explicit solvers.
     *
     * Contains configuration parameters for explicit ODE solvers, including the solver method
     * (Runge-Kutta, Adams-Bashforth) and coefficients.
     */
    struct ExplicitSettings {
        str method;                            //!< The explicit solver method (e.g., Runge-Kutta, Adams-Bashforth).
        std::optional<int> RungeKutta_order;   //!< Order of the Runge-Kutta method, if applicable.
        std::optional<Eigen::MatrixXd> RungeKutta_coefficients_a; //!< Coefficients \(a_{ij}\) for Runge-Kutta methods.
        std::optional<Eigen::VectorXd> RungeKutta_coefficients_b; //!< Coefficients \(b_i\) for Runge-Kutta methods.
        std::optional<Eigen::VectorXd> RungeKutta_coefficients_c; //!< Coefficients \(c_i\) for Runge-Kutta methods.
        std::optional<int> AdamsBashforth_max_order; //!< Maximum order for Adams-Bashforth methods.
        std::optional<Eigen::VectorXd> AdamsBashforth_coefficients_vector; //!< Coefficients for Adams-Bashforth methods.
    };

    /**
     * @struct ImplicitSettings
     * @brief Holds settings for implicit solvers.
     *
     * Contains configuration parameters for implicit solvers, including solver methods,
     * convergence criteria, and options for linear and nonlinear system representations.
     */
    struct ImplicitSettings {
        str method;                            //!< The implicit solver method (e.g., Backward Euler, Crank-Nicolson).
        bool rhs_is_linear;                    //!< Flag indicating whether the right-hand side is linear.
        std::optional<str> linear_system_solver; //!< Name of the solver for linear systems.
        std::optional<LinearSystem> rhs_system; //!< The linear system representing the RHS, if applicable.
        std::optional<double> tolerance;       //!< Convergence tolerance for nonlinear solvers.
        std::optional<int> max_iterations;     //!< Maximum iterations for nonlinear solvers.
        std::optional<double> dx;              //!< Step size for numerical differentiation.
        std::optional<str> root_finder;        //!< Name of the root-finding method for nonlinear solvers.
    };

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
