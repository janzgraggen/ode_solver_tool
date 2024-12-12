/**
 * @file OdeSolver.cc
 * @brief Implementation of the OdeSolver class for solving ODEs.
 *
 * This file contains the implementation of the `OdeSolver` class, which serves as a base class
 * for solving ordinary differential equations (ODEs). It provides functionality to set and
 * manage integration step size, time intervals, and the system's right-hand side function.
 * The solver supports output logging to CSV files and integrates various solver components,
 * such as configuration management and function evaluation.
 *
 * The class relies on the `Logger` for logging and the `CSVWriter` for result output.
 *
 * Author: natha
 * Date: 25/11/2024
 */

#include "OdeSolver.hh"
#include "../Writer/CSVWriter.hh"
#include "../Utils/SettingsStruct.hh"

using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;
using str = std::string;

/**
 * @brief Default constructor for the OdeSolver class.
 *
 * Initializes the ODE solver with default values for step size, initial time, and final time.
 * Sets up the logger for tracking solver operations.
 *
 * @param logger_ Reference to the `Logger` object.
 */
OdeSolver::OdeSolver(Logger& logger_) : logger(&logger_), stepSize(0.0), initialTime(0.0), finalTime(0.0) {}

/**
 * @brief Virtual destructor for OdeSolver.
 *
 * Ensures proper cleanup in derived classes.
 */
OdeSolver::~OdeSolver() = default;

/**
 * @brief Sets the step size for the ODE solver.
 *
 * Defines the step size \( h \) that determines the increment in time steps during integration.
 *
 * @param h The step size to be set for numerical integration.
 */
void OdeSolver::setStepSize(const double h) {
    stepSize = h;
}

/**
 * @brief Sets the output file name for storing the ODE solver's results.
 *
 * Specifies where the solver's intermediate and final results will be stored in CSV format.
 *
 * @param filename The desired output file name.
 */
void OdeSolver::setOutputFileName(const str& filename) {
    outputFileName = filename;
}

/**
 * @brief Retrieves the output file name for the ODE solver.
 *
 * Returns the filename where ODE integration results are stored.
 *
 * @return The output filename as a string.
 */
str OdeSolver::getOutputFileName() const {
    return outputFileName;
}

/**
 * @brief Retrieves the step size currently used by the ODE solver.
 *
 * Returns the step size configured for the solver's integration process.
 *
 * @return The step size.
 */
double OdeSolver::getStepSize() const {
    return stepSize;
}

/**
 * @brief Sets the time interval for the ODE solver.
 *
 * Defines the initial and final times for the integration process.
 *
 * @param t0 The initial time of the interval.
 * @param t1 The final time of the interval.
 */
void OdeSolver::setTimeInterval(const double t0, const double t1) {
    initialTime = t0;
    finalTime = t1;
}

/**
 * @brief Gets the initial time of the ODE solver.
 *
 * Returns the configured initial time of the solver.
 *
 * @return The initial time.
 */
double OdeSolver::getInitialTime() const {
    return initialTime;
}

/**
 * @brief Gets the final time of the ODE solver.
 *
 * Returns the configured final time of the solver.
 *
 * @return The final time.
 */
double OdeSolver::getFinalTime() const {
    return finalTime;
}

/**
 * @brief Sets the initial value (state vector) for the ODE solver.
 *
 * Provides the solver with an initial state to start the integration process.
 *
 * @param y0 The initial state vector represented as an Eigen vector.
 */
void OdeSolver::setInitialValue(const Eigen::VectorXd& y0) {
    initialValue = y0;
}

/**
 * @brief Sets the global configuration for the ODE solver from a `Reader` object.
 *
 * Loads integration step size, time intervals, initial values, and the right-hand side function
 * from a provided `Reader` configuration.
 *
 * @param Rdr The `Reader` object containing the ODE configuration.
 */
void OdeSolver::setGlobalConfig(const Reader& Rdr) {
    setOutputFileName(Rdr.getOutputFileName());
    OdeSettings settings = Rdr.getOdeSettings();
    if (settings.step_size <= 0.0  || settings.step_size > (settings.final_time - settings.initial_time)) {
        logger->error("{in OdeSolver::setGlobalConfig()} Invalid step size: " + std::to_string(settings.step_size));
    } else {
        setStepSize(settings.step_size);
    }
    if (settings.initial_time >= settings.final_time) {
        logger->error("{in OdeSolver::setGlobalConfig()} Invalid time interval: [" + std::to_string(settings.initial_time) + ", " + std::to_string(settings.final_time) + "]");
    } else {
        setTimeInterval(settings.initial_time, settings.final_time);
        setInitialValue(settings.initial_value);
    }
    int iters = std::floor((settings.final_time- settings.initial_time)/settings.step_size);
    if (iters > 1e6) {
        logger->warning("{in OdeSolver::setGlobalConfig()} Large number of steps: " + std::to_string(iters));
    }
    setRightHandSide(Rdr.getFunction());
}

/**
 * @brief Gets the initial value (state vector) of the ODE solver.
 *
 * Returns the state vector that serves as the initial condition for the integration process.
 *
 * @return The initial value as an Eigen vector.
 */
Eigen::VectorXd OdeSolver::getInitialValue() const {
    return initialValue;
}

/**
 * @brief Sets the right-hand side function of the ODE system.
 *
 * Associates the system's right-hand side function \( f(y, t) \) with the solver.
 *
 * @param f The right-hand side function represented as an `f_TYPE` lambda or function object.
 */
void OdeSolver::setRightHandSide(const f_TYPE& f) {
    f_rhs = f;
}

/**
 * @brief Gets the right-hand side function of the ODE system.
 *
 * Returns the function representing the system of equations \( \dot{y} = f(y, t) \).
 *
 * @return The system's right-hand side function.
 */
f_TYPE OdeSolver::getRightHandSide() const {
    return f_rhs;
}

/**
 * @brief Solves the ODE system over the specified time range.
 *
 * Iteratively advances the solution by calling `Step` until the final time is reached.
 * Logs intermediate results to a CSV file using the `CSVWriter`.
 *
 * @return The solution vector at the final time step.
 */
Eigen::VectorXd OdeSolver::solveOde() {

    Eigen::VectorXd y = getInitialValue();
    double t = getInitialTime();
    CSVWriter writer(getOutputFileName());
    writer.writeHeader(y);
    writer.writeLine(t, y);

    while (t <= getFinalTime()) {
        y = calcStep(y, t);
        t += getStepSize();
        writer.writeLine(t, y);
    }

    return y;
}
