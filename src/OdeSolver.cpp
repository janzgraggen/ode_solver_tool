//
// Created by natha on 25/11/2024.
//

#include "OdeSolver.hpp"

/**
 * @brief Default constructor for ODE_Solver.
 *
 * Initializes the step size, initial time, and final time to zero.
 */
ODE_Solver::ODE_Solver() : stepSize(0.0), initialTime(0.0), finalTime(0.0) {}

/**
 * @brief Virtual destructor for ODE_Solver.
 *
 * This ensures proper cleanup in derived classes.
 */
ODE_Solver::~ODE_Solver() = default;

/**
 * @brief Sets the step size for the ODE solver.
 *
 * @param h The step size for numerical integration.
 */
void ODE_Solver::SetStepSize(const double h) {
    stepSize = h;
}

/**
 * @brief Gets the step size used by the ODE solver.
 *
 * @return The current step size.
 */
double ODE_Solver::GetStepSize() const {
    return stepSize;
}

/**
 * @brief Sets the time interval for the ODE solver.
 *
 * @param t0 The initial time of the interval.
 * @param t1 The final time of the interval.
 */
void ODE_Solver::SetTimeInterval(const double t0, const double t1) {
    initialTime = t0;
    finalTime = t1;
}

/**
 * @brief Gets the initial time of the ODE solver.
 *
 * @return The initial time.
 */
double ODE_Solver::GetInitialTime() const {
    return initialTime;
}

/**
 * @brief Gets the final time of the ODE solver.
 *
 * @return The final time.
 */
double ODE_Solver::GetFinalTime() const {
    return finalTime;
}

/**
 * @brief Sets the initial value (state vector) for the ODE solver.
 *
 * @param y0 The initial value as an Eigen vector.
 */
void ODE_Solver::SetInitialValue(const Eigen::VectorXd& y0) {
    initialValue = y0;
}

/**
 * @brief Gets the initial value (state vector) of the ODE solver.
 *
 * @return The initial value as an Eigen vector.
 */
Eigen::VectorXd ODE_Solver::GetInitialValue() const {
    return initialValue;
}

/**
 * @brief Sets the right-hand side function of the ODE.
 *
 * The function should represent the system of equations \( \dot{y} = f(y, t) \).
 *
 * @param f A function that computes the right-hand side, taking the state vector
 * and time as inputs, and returning the derivative as an Eigen vector.
 */
void ODE_Solver::SetRightHandSide(const std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>& f) {
    f_rhs = f;
}

/**
 * @brief Gets the right-hand side function of the ODE.
 *
 * @return The right-hand side function as a `std::function`.
 */
std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> ODE_Solver::GetRightHandSide() const {
    return f_rhs;
}