//
// Created by natha on 25/11/2024.
//

#ifndef EXPLICIT_HH
#define EXPLICIT_HH

#include "OdeSolver.hh"

/**
 * @class Explicit
 * @brief A base class for explicit numerical methods for solving ODEs.
 *
 * This class extends `ODE_Solver` to provide a structure for explicit methods,
 * where derived classes implement the step logic for updating the solution.
 */
class Explicit : public OdeSolver {
public:
    /**
     * @brief Default virtual destructor for the Explicit class.
     *
     * Ensures proper cleanup in derived classes.
     */
    ~Explicit() override = default;

    /**
     * @brief Solves the ODE using an explicit method.
     *
     * Encapsulates the loop for integrating the ODE from the initial time to the final time
     * using the step logic defined in derived classes. Writes the results to the given output stream.
     *
     * @param stream The output stream to write results (e.g., `std::cout` or a file stream).
     */
    Eigen::VectorXd SolveODE(std::ostream& stream) override;
};

#endif // EXPLICIT_HH
