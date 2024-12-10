/**
* @file Explicit.cc
 * @brief Implementation file for the Explicit class.
 *
 * This file contains the implementation of the `Explicit` class, which serves
 * as a base for explicit numerical methods to solve ordinary differential equations (ODEs).
 * The class extends `OdeSolver` and is designed to be subclassed by specific explicit schemes.
 *
 * Author: natha
 * Date: 25/11/2024
 */

#include "Explicit.hh"

/**
 * @brief Constructs the Explicit solver.
 *
 * Initializes the solver with a logger for debugging and recording messages.
 *
 * @param logger_ A `Logger` object used for logging.
 */
Explicit::Explicit(Logger& logger_) : OdeSolver(logger_) {}