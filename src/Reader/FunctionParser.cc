#include "FunctionParser.hh"
#include <stdexcept>
#include <iostream>

FunctionParser::FunctionParser(const strList>& functions)
    : f_i(functions), n(functions.size()) {
    if (n == 0) {
        throw std::invalid_argument("The function vector must not be empty.");
    }

    // Initialize shared variables for parsers
    variables.resize(n + 1); // First variable for t, rest for y1, ..., yn

    // Initialize parsers
    parsers.resize(n);
    for (size_t i = 0; i < n; ++i) {
        parsers[i].DefineVar("t", &variables[0]); // Time variable
        for (size_t j = 0; j < n; ++j) {
            parsers[i].DefineVar("y" + std::to_string(j + 1), &variables[j + 1]); // y1, ..., yn
        }

        try {
            parsers[i].SetExpr(functions[i]); // Set the function expression
        } catch (const mu::Parser::exception_type& e) {
            throw std::invalid_argument("Error in parsing function: " + std::string(e.GetMsg()));
        }
    }
}

Eigen::VectorXd FunctionParser::evaluate(const Eigen::VectorXd& input) {
    if (input.size() != n + 1) {
        throw std::invalid_argument("Input vector size must be " + std::to_string(n + 1) + ".");
    }

    // Assign input to variables
    variables[0] = input[0]; // t
    for (size_t i = 1; i <= n; ++i) {
        variables[i] = input[i];
    }

    // Evaluate each function
    Eigen::VectorXd output(n);
    for (size_t i = 0; i < n; ++i) {
        try {
            output[i] = parsers[i].Eval();
        } catch (const mu::Parser::exception_type& e) {
            throw std::runtime_error("Error during evaluation: " + std::string(e.GetMsg()));
        }
    }

    return output;
}

f_TYPE FunctionParser::getFunction() const {
    return [this](const Eigen::VectorXd& y,double t) {
        if (y.size() != n) {
            throw std::invalid_argument("Input vector y size must be " + std::to_string(n) + ".");
        }

        // Combine t and y into a single vector
        Eigen::VectorXd input(n + 1);
        input[0] = t; // Assign t to the first position
        input.tail(n) = y; // Assign y1, ..., yn to the rest of the vector

        // Evaluate the function
        return this->evaluate(input);
    };
}
