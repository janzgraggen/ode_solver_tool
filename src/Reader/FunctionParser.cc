/**
 * @file FunctionParser.cc
 * @brief Implementation of the FunctionParser class for parsing and evaluating mathematical functions.
 *
 * This source file contains the implementation of the FunctionParser class, which supports parsing,
 * evaluating, and retrieving mathematical functions represented as strings. The class utilizes the
 * muParser library to handle mathematical expressions and integrates with the Eigen library for vector
 * operations. It provides functionality to bind variables dynamically and evaluate multiple functions
 * efficiently while ensuring proper error handling.
 *
 * Author: janzgraggen
 */

#include "FunctionParser.hh"
#include <stdexcept>

 /**
  * @brief Constructor for the FunctionParser class.
  * 
  * Initializes the FunctionParser with a specified dimension `n` and a list of functions.
  * Sets up the parsers for each function, binds variables to shared storage, and parses the provided
  * mathematical expressions.
  *
  * @param n Number of functions to parse and evaluate.
  * @param functions A list of strings representing mathematical functions.
  * @throws std::invalid_argument if `n` is zero or the number of provided functions does not match `n`.
  */
FunctionParser::FunctionParser(int n, const strList& functions)
    : f_i(functions), n(n) {
    if (n == 0) {
        throw std::invalid_argument("The dimension (n) must not be zero.");
    }
    if (functions.size() != n) {
        throw std::invalid_argument("The number of functions does not match the specified dimension (n).");
    }

    // Initialize shared variables for parsers
    variables.resize(n + 1); // First variable for time 't', the rest for y1, ..., yn
    parsers.resize(n);

    for (size_t i = 0; i < n; ++i) {
        // Define the "t" variable for time binding
        parsers[i].DefineVar("t", &variables[0]);
        // Define the y1, ..., yn variables for dynamic binding
        for (size_t j = 0; j < n; ++j) {
            parsers[i].DefineVar("y" + std::to_string(j + 1), &variables[j + 1]);
        }

        try {
            parsers[i].SetExpr(functions[i]); // Set the function expression
        } catch (const mu::Parser::exception_type& e) {
            throw std::invalid_argument("Error in parsing function: " + std::string(e.GetMsg()));
        }
    }
}

/**
 * @brief Evaluates the stored functions given an input vector.
 *
 * Takes an input vector containing time `t` and state variables `y1, ..., yn` and evaluates the stored
 * functions by binding each value to the shared parser variables. It then returns an output vector of
 * evaluated functions.
 *
 * @param input An Eigen::VectorXd containing input values [t, y1, ..., yn].
 * @return Eigen::VectorXd containing the evaluated functions.
 * @throws std::invalid_argument if the input vector size does not match `n + 1`.
 * @throws std::runtime_error if an error occurs during function evaluation.
 */
Eigen::VectorXd FunctionParser::evaluate(const Eigen::VectorXd& input) {
    if (input.size() != n + 1) {
        throw std::invalid_argument("Input vector size must be " + std::to_string(n + 1) + ".");
    }

    // Bind input values to the shared variables
    variables[0] = input[0]; // Time 't'
    for (size_t i = 1; i <= n; ++i) {
        variables[i] = input[i]; // State variables y1, y2, ..., yn
    }

    // Evaluate each function and store the results
    Eigen::VectorXd output(n);
    for (size_t i = 0; i < n; ++i) {
        try {
            // Bind the "t" variable
            parsers[i].DefineVar("t", &variables[0]);
            // Bind y1, ..., yn dynamically
            for (size_t j = 0; j < n; ++j) {
                parsers[i].DefineVar("y" + std::to_string(j + 1), &variables[j + 1]);
            }

            output[i] = parsers[i].Eval();
        } catch (const mu::Parser::exception_type& e) {
            throw std::runtime_error("Error during evaluation: " + std::string(e.GetMsg()));
        }
    }
    return output;
}

/**
 * @brief Returns a lambda function to evaluate functions dynamically.
 *
 * Generates a lambda that captures the parsers and shared variables by value. The lambda takes time
 * `t` and state vector `y` as inputs, updates the shared variables, and evaluates the stored functions.
 *
 * @return f_TYPE A lambda function that evaluates the functions given a state vector `y` and time `t`.
 * @throws std::invalid_argument if the input state vector `y` size does not match `n`.
 * @throws std::runtime_error if an error occurs during function evaluation.
 */
f_TYPE FunctionParser::getFunction() {
    auto parsers_copy = parsers;
    auto n_copy = n;

    return [parsers_copy, n_copy, variables = this->variables](const Eigen::VectorXd& y, double t) mutable {
        if (y.size() != n_copy) {
            throw std::invalid_argument("Input vector y size must be " + std::to_string(n_copy) + ".");
        }

        // Combine time `t` and state variables `y` into a single input vector
        Eigen::VectorXd input(n_copy + 1);
        input[0] = t;
        input.tail(n_copy) = y;

        // Update shared variables with time and state values
        variables[0] = t;
        for (size_t i = 1; i <= n_copy; ++i) {
            variables[i] = input[i];
        }

        for (size_t i = 0; i < n_copy; ++i) {
            parsers_copy[i].DefineVar("t", &variables[0]);
            for (size_t j = 0; j < n_copy; ++j) {
                parsers_copy[i].DefineVar("y" + std::to_string(j + 1), &variables[j + 1]);
            }
        }

        Eigen::VectorXd output(n_copy);
        for (size_t i = 0; i < n_copy; ++i) {
            try {
                output[i] = parsers_copy[i].Eval();
            } catch (const mu::Parser::exception_type& e) {
                throw std::runtime_error("Error during evaluation: " + std::string(e.GetMsg()));
            }
        }
        return output;
    };
}