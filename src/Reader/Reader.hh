#ifndef __READER_HH__
#define __READER_HH__

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <optional>
#include "../Utils/LinSysStruct.hh"

/**
 * @file Reader.hh
 * @brief Defines the `Reader` class for parsing ODE solver settings from a YAML file.
 */

using str = std::string;
using strList = std::vector<std::string>;
using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;

/**
 * @class Reader
 * @brief Parses configuration settings for ODE solvers from a YAML file.
 *
 * This class provides methods to retrieve general ODE solver settings,
 * as well as specific settings for explicit and implicit solvers.
 */
class Reader {
public:
    /**
     * @brief Constructs a `Reader` object and loads the specified YAML file.
     * @param filename The path to the YAML configuration file.
     * @throws std::runtime_error If the file cannot be opened or parsed.
     */
    Reader(const str& filename);

    /**
     * @brief Retrieves the solver type specified in the configuration.
     * @return A string indicating the solver type (e.g., "Implicit" or "Explicit").
     */
    str getSolverType() const;
    str getOutputFileName() const;
    int getDim() const;
    strList getFunctionStringlist() const;
    f_TYPE getFunction() const;
    

    /**
     * @struct OdeSettings
     * @brief Holds general settings for ODE solvers.
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
     * @brief Retrieves the general ODE solver settings.
     * @return An `OdeSettings` structure containing the general settings.
     */
    OdeSettings getOdeSettings() const;

    /**
     * @brief Retrieves the settings for explicit solvers.
     * @return An `ExplicitSettings` structure containing the explicit solver settings.
     */
    ExplicitSettings getExplicitSettings() const;

    /**
     * @brief Retrieves the settings for implicit solvers.
     * @return An `ImplicitSettings` structure containing the implicit solver settings.
     */
    ImplicitSettings getImplicitSettings() const;

private:
    YAML::Node config; //!< The YAML configuration object.
};

#endif // __READER_HH__
