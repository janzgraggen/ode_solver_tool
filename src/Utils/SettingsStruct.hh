/**
 * @file SettingsStruct.hh
 * @brief Defines the `OdeSettings`, `ExplicitSettings` and `ImplicitSettings` structs representing the settings parameters set by the configuration.
 *
 * 
 * @author: janzgraggen
 * @date: 11/12/2024
 */

#ifndef SETTINGS_STRUCT_HH
#define SETTINGS_STRUCT_HH

#include <Eigen/Dense>
#include "LinSysStruct.hh"


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
    std::string method;                            //!< The explicit solver method (e.g., Runge-Kutta, Adams-Bashforth).
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
    std::string method;                            //!< The implicit solver method (e.g., Backward Euler, Crank-Nicolson).
    bool rhs_is_linear;                    //!< Flag indicating whether the right-hand side is linear.
    std::optional<std::string> linear_system_solver; //!< Name of the solver for linear systems.
    std::optional<LinearSystem> rhs_system; //!< The linear system representing the RHS, if applicable.
    std::optional<double> tolerance;       //!< Convergence tolerance for nonlinear solvers.
    std::optional<int> max_iterations;     //!< Maximum iterations for nonlinear solvers.
    std::optional<double> dx;              //!< Step size for numerical differentiation.
    std::optional<std::string> root_finder;        //!< Name of the root-finding method for nonlinear solvers.
};

#endif // SETTINGS_STRUCT_HH