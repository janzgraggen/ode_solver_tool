/**
* @file Explicit.hh
 * @brief Header file for the Explicit class.
 *
 * This header defines the `Explicit` class, which serves as a base for explicit
 * numerical methods to solve ordinary differential equations (ODEs).
 * Derived classes must implement specific explicit schemes by overriding the
 * appropriate methods. The class builds upon the functionality of `OdeSolver`.
 *
 * Author: natha
 * Date: 25/11/2024
 */

#ifndef EXPLICIT_HH
#define EXPLICIT_HH

#include "OdeSolver.hh"

/**
 * @class Explicit
 * @brief A base class for explicit numerical methods for solving ODEs.
 *
 * The `Explicit` class is a foundation for implementing numerical methods
 * that use explicit schemes. These schemes calculate the state of a system
 * at the next time step using only information from the current time step.
 *
 * This class inherits from `OdeSolver` and is intended to be subclassed.
 * Derived classes should implement the `calcStep` method to define the explicit
 * time-stepping algorithm.
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
   * @brief Constructs the Explicit solver.
   *
   * Initializes the solver with a logger for recording messages and errors.
   *
   * @param logger_ A `Logger` object used for debugging and logging.
   */
  Explicit(Logger& logger_);
};

#endif // EXPLICIT_HH
