//
// Created by natha on 25/11/2024.
//
#ifndef ODE_SOLVER_HPP
#define ODE_SOLVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <functional>

/**
 * @class ODE_Solver
 * @brief A base class for solving Ordinary Differential Equations (ODEs).
 *
 * This class provides an interface for solving ODEs using different numerical methods.
 * It uses Eigen for vector representation and supports customizable right-hand side functions.
 */
class ODE_Solver {
private:
    double stepSize; ///< The step size for the numerical method.
    double initialTime; ///< The initial time of the simulation.
    double finalTime; ///< The final time of the simulation.
    Eigen::VectorXd initialValue; ///< The initial state vector of the system.
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> f_rhs; ///< The right-hand side function of the ODE.

public:
    /**
     * @brief Default constructor for the ODE_Solver class.
     */
    ODE_Solver();

    /**
     * @brief Virtual destructor for the ODE_Solver class.
     */
    virtual ~ODE_Solver();

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
    void SetRightHandSide(const std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>& f);

    /**
     * @brief Gets the right-hand side function of the ODE.
     * @return The right-hand side function.
     */
    [[nodiscard]] std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> GetRightHandSide() const;

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

#endif // ODE_SOLVER_HPP
