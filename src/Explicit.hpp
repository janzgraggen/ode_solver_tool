//
// Created by natha on 25/11/2024.
//

#ifndef EXPLICIT_HPP
#define EXPLICIT_HPP

#include "OdeSolver.hpp"

/**
 * @class Explicit
 * @brief A base class for explicit numerical methods for solving ODEs.
 *
 * This class extends `ODE_Solver` to provide a structure for explicit methods,
 * where derived classes implement the step logic for updating the solution.
 */
class Explicit : public ODE_Solver {
public:
    /**
     * @brief Default virtual destructor for the Explicit class.
     *
     * Ensures proper cleanup in derived classes.
     */
    ~Explicit() override = default;

    /**
     * @brief Computes a single explicit step of the ODE solver.
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
     * @brief Solves the ODE using an explicit method.
     *
     * Encapsulates the loop for integrating the ODE from the initial time to the final time
     * using the step logic defined in derived classes. Writes the results to the given output stream.
     *
     * @param stream The output stream to write results (e.g., `std::cout` or a file stream).
     */
    void SolveODE(std::ostream& stream) override;
};

#endif // EXPLICIT_HPP
