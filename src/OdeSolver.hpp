//
// Created by natha on 25/11/2024.
//
#ifndef ODESOLVER_HPP
#define ODESOLVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <functional>

using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;
/**
 * @class OdeSolver
 * @brief A base class for solving Ordinary Differential Equations (ODEs).
 *
 * This class provides an interface for solving ODEs using different numerical methods.
 * It uses Eigen for vector representation and supports customizable right-hand side functions.
 */
class OdeSolver {
private:
    double stepSize; ///< The step size for the numerical method.
    double initialTime; ///< The initial time of the simulation.
    double finalTime; ///< The final time of the simulation.
    Eigen::VectorXd initialValue; ///< The initial state vector of the system.

protected: //need protedted to be able do assign different functions (linear/nonlinear in child classes)
    f_TYPE f_rhs; ///< The right-hand side function of the ODE.

public:
    /**
     * @brief Default constructor for the OdeSolver class.
     */
    OdeSolver();

    /**
     * @brief Virtual destructor for the OdeSolver class.
     */
    virtual ~OdeSolver();

    /**
     * @brief Sets the step size for the ODE solver.
     * @param h The step size.
     */
    void SetStepSize(double h);

    /**
     * @brief Gets the step size of the ODE solver.
     * @return The step size.
     */
    [[nodiscard]] double GetStepSize() const;

    /**
     * @brief Sets the time interval for the ODE solver.
     * @param t0 The initial time.
     * @param t1 The final time.
     */
    void SetTimeInterval(double t0, double t1);

    /**
     * @brief Gets the initial time of the ODE solver.
     * @return The initial time.
     */
    [[nodiscard]] double GetInitialTime() const;

    /**
     * @brief Gets the final time of the ODE solver.
     * @return The final time.
     */
    [[nodiscard]] double GetFinalTime() const;

    /**
     * @brief Sets the initial value (state vector) for the ODE solver.
     * @param y0 The initial value.
     */
    void SetInitialValue(const Eigen::VectorXd& y0);

    /**
     * @brief Gets the initial value (state vector) of the ODE solver.
     * @return The initial state vector.
     */
    [[nodiscard]] Eigen::VectorXd GetInitialValue() const;

    /**
     * @brief Sets the right-hand side function of the ODE.
     * @param f A function representing the right-hand side of the ODE: \( f(y, t) \).
     */
    virtual void SetRightHandSide(const f_TYPE& f);

    /**
     * @brief Gets the right-hand side function of the ODE.
     * @return The right-hand side function.
     */
    [[nodiscard]] f_TYPE GetRightHandSide() const;

    /**
     * @brief Computes a single step of the ODE solver.
     *
     * Derived classes must implement this method to define the logic for advancing
     * the solution by one step.
     *
     * @param y The current state vector.
     * @param t The current time.
     * @return The state vector after one step.
     */
    virtual Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) = 0;


    /**
     * @brief Pure virtual function for solving the ODE.
     *
     * This function must be implemented by derived classes to solve the ODE and output
     * the results.
     *
     * @param stream The output stream to which results are written.
     */
    virtual void SolveODE(std::ostream& stream) = 0;
};

 #endif // ODESOLVER_HPP
