/**
 * @file OdeSolver.hh
 * @brief Header file for the OdeSolver class for solving Ordinary Differential Equations (ODEs).
 *
 * This header file defines the interface for the `OdeSolver` class, which serves as a base class
 * for solving ordinary differential equations (ODEs). The `OdeSolver` class provides methods to set
 * and manage integration step size, time intervals, and the system's right-hand side function.
 * It supports customizable right-hand side functions, vector representations using the Eigen library,
 * and output logging through the `Logger` and `Reader` components.
 *
 * The class is designed to be extended by derived classes, allowing for the implementation of various
 * numerical integration methods (e.g., Euler, Runge-Kutta).
 *
 * Author: natha
 * Date: 25/11/2024
 */

#ifndef ODESOLVER_HH
#define ODESOLVER_HH

#include <functional>
#include "../src/Reader/Reader.hh"
#include "../Logger/Logger.hh"

using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>; ///< Alias for the ODE right-hand side function type.
using str = std::string; ///< Alias for the string type.

/**
 * @class OdeSolver
 * @brief A base class for solving Ordinary Differential Equations (ODEs).
 *
 * The `OdeSolver` class provides an interface to solve ODEs using different numerical methods.
 * It leverages the Eigen library to represent vectors and allows users to customize the system's
 * right-hand side function. The class supports configurable integration step size, time intervals,
 * and output logging to facilitate debugging and analysis of solver results.
 */
class OdeSolver {
private:
    double stepSize; ///< The step size for the numerical method.
    double initialTime; ///< The initial time of the simulation.
    double finalTime; ///< The final time of the simulation.
    Eigen::VectorXd initialValue; ///< The initial state vector of the system.
    str OutputFileName; ///< The output file name for storing results.

protected: // Protected access to allow assignment of different right-hand side functions in derived classes.
    f_TYPE f_rhs; ///< The right-hand side function of the ODE system.

public:
    Logger logger; ///< The logger object for logging messages and solver activities.

    /**
     * @brief Default constructor for the OdeSolver class.
     * Initializes step size, initial time, and final time to zero.
     *
     * @param logger_ Reference to a `Logger` object.
     */
    OdeSolver(Logger& logger_);

    /**
     * @brief Virtual destructor for the OdeSolver class.
     * Ensures proper cleanup in derived classes.
     */
    virtual ~OdeSolver();

    /**
     * @brief Sets the step size for the ODE solver.
     * Defines the step size \( h \) for numerical integration.
     *
     * @param h The step size value.
     */
    void SetStepSize(double h);

    /**
     * @brief Sets the output file name for storing the ODE solver results.
     *
     * Specifies the name of the CSV file where intermediate and final integration results will be saved.
     *
     * @param filename The desired output file name.
     */
    void SetOutputFileName(const str& filename);

    /**
     * @brief Retrieves the output file name for the ODE solver.
     *
     * @return The filename where solver results are stored.
     */
    [[nodiscard]] str GetOutputFileName() const;

    /**
     * @brief Retrieves the step size currently in use by the ODE solver.
     *
     * @return The integration step size.
     */
    [[nodiscard]] double GetStepSize() const;

    /**
     * @brief Sets the time interval for the ODE solver.
     *
     * Configures the time range over which the solver integrates the system.
     *
     * @param t0 The starting time.
     * @param t1 The ending time.
     */
    void SetTimeInterval(double t0, double t1);

    /**
     * @brief Sets the global parameters for the ODE solver using a `Reader` object.
     *
     * Loads integration step size, time intervals, initial conditions, and the right-hand side function
     * from the provided `Reader` configuration.
     *
     * @param Rdr The `Reader` object containing solver parameters.
     */
    virtual void SetGlobalConfig(const Reader& Rdr);

    /**
     * @brief Sets the specific solver configuration for ODE methods.
     *
     * Derived classes must implement this method to set up solver-specific parameters.
     *
     * @param Rdr The `Reader` object containing method-specific configurations.
     */
    virtual void SetConfig(const Reader& Rdr) = 0;

    /**
     * @brief Retrieves the initial time of the ODE solver.
     *
     * @return The initial time.
     */
    [[nodiscard]] double GetInitialTime() const;

    /**
     * @brief Retrieves the final time of the ODE solver.
     *
     * @return The final time.
     */
    [[nodiscard]] double GetFinalTime() const;

    /**
     * @brief Sets the initial state vector for the ODE solver.
     *
     * Provides the solver with the system's initial conditions for starting the integration process.
     *
     * @param y0 The initial state vector.
     */
    void SetInitialValue(const Eigen::VectorXd& y0);

    /**
     * @brief Retrieves the initial state vector of the ODE solver.
     *
     * @return The initial state vector representing the solver's starting conditions.
     */
    [[nodiscard]] Eigen::VectorXd GetInitialValue() const;

    /**
     * @brief Sets the right-hand side function representing the system's ODEs.
     *
     * Associates a custom right-hand side function \( f(y, t) \) with the solver.
     *
     * @param f The right-hand side function as an `f_TYPE` lambda or function object.
     */
    virtual void SetRightHandSide(const f_TYPE& f);

    /**
     * @brief Retrieves the right-hand side function representing the system of ODEs.
     *
     * @return The system's right-hand side function.
     */
    [[nodiscard]] f_TYPE GetRightHandSide() const;

    /**
     * @brief Computes a single step of the solver's integration process.
     *
     * This method must be implemented by derived classes to define the logic for advancing
     * the system's state by a single step in time.
     *
     * @param y The current system state vector.
     * @param t The current time.
     * @return The system state vector after performing one integration step.
     */
    virtual Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) = 0;

    /**
     * @brief Solves the ODE system across the configured time interval.
     *
     * Calls the `Step` method repeatedly until the final time is reached.
     * Outputs intermediate states to an output stream or storage mechanism.
     *
     * @return The state vector of the system at the final time step.
     */
    Eigen::VectorXd SolveODE();
};

#endif // ODESOLVER_HH