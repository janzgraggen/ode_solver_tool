/**
 * @file FunctionParser.hh
 * @brief Header file for the FunctionParser class, which parses and evaluates mathematical functions.
 *
 * This header defines the `FunctionParser` class, which provides functionality to parse, evaluate,
 * and dynamically bind mathematical functions represented as strings. It leverages the muParser library
 * for mathematical expression parsing and the Eigen library for efficient vector operations.
 *
 * The class supports multiple functions, dynamically binds time and state variables, and ensures proper
 * error handling and evaluation of mathematical expressions for scalable ODE-solving and simulation tasks.
 *
 * Author: [Your Name]  
 * Date: [YYYY-MM-DD]
 */

#ifndef __FUNCTIONPARSER_HH__
#define __FUNCTIONPARSER_HH__

#include <vector>
#include <string>
#include <functional>
#include <Eigen/Dense>
#include "muParser.h"

using strList = std::vector<std::string>;     ///< A list of strings representing mathematical functions.
using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>; ///< A callable type representing a function that takes a state vector and time as input.

class FunctionParser {
private:
    strList f_i;             ///< A list of function strings representing mathematical expressions.
    std::vector<mu::Parser> parsers;          ///< muParser instances, one for each mathematical function.
    std::vector<double> variables;            ///< Shared storage for parser variables (time 't', and state variables y1, ..., yn).
    size_t n;                                 ///< The dimension of the input and output vectors.

public:
    /**
     * @brief Constructor for the FunctionParser class.
     * 
     * Initializes the FunctionParser with the specified dimension `n` and a list of functions.
     * Sets up the parsers for each function, binds shared variables to the parsers, and parses the provided
     * mathematical expressions.
     *
     * @param n Number of functions to parse and evaluate.
     * @param functions A list of strings representing mathematical functions.
     * @throws std::invalid_argument if `n` is zero or if the provided number of functions does not match `n`.
     */
    FunctionParser(int n, const strList& functions);

    /**
     * @brief Destructor for the FunctionParser class.
     *
     * Cleans up resources when the FunctionParser object is destroyed.
     */
    ~FunctionParser() = default;

    /**
     * @brief Evaluates the stored functions at a given input vector.
     *
     * Takes an input vector containing time `t` and state variables `y1, ..., yn` and evaluates the functions
     * by dynamically binding these values to the shared parser variables. Returns the computed output vector.
     *
     * @param input An `Eigen::VectorXd` containing [t, y1, ..., yn].
     * @return An `Eigen::VectorXd` containing the evaluated functions.
     * @throws std::invalid_argument if the input vector size does not match `n + 1`.
     * @throws std::runtime_error if any error occurs during function evaluation.
     */
    Eigen::VectorXd evaluateParsedFunction(const Eigen::VectorXd& input);

    /**
     * @brief Returns a callable function that evaluates the stored functions dynamically.
     *
     * Generates a lambda function that captures the parsers and shared variables. The lambda takes time `t`
     * and a state vector `y` as inputs, updates the shared variables, and evaluates the functions.
     *
     * @return A callable `f_TYPE` lambda function that evaluates the functions given a state vector `y` and time `t`.
     * @throws std::invalid_argument if the size of the input state vector `y` does not match the dimension `n`.
     * @throws std::runtime_error if any errors occur during function evaluation.
     */
    f_TYPE getFunction();

    /**
     * @brief Returns the number of functions currently stored.
     *
     * @return The dimension `n` representing the number of functions.
     */
    size_t getDimension() const { return n; }
};

#endif // __FUNCTIONPARSER_HH__