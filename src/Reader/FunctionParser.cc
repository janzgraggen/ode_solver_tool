#include "FunctionParser.hh"
#include <stdexcept>

FunctionParser::FunctionParser(int n, const strList& functions)
    : f_i(functions), n(n) {
    if (n == 0) {
        throw std::invalid_argument("The dimension (n) must not be zero.");
    }
    if (functions.size() != n) {
        throw std::invalid_argument("The number of functions does not match the specified dimension (n).");
    }

    // std::cout << "Number of functions (n): " << n << std::endl;
    // std::cout << "FunctionParser initiated" << std::endl;

    // Initialize shared variables for parsers
    variables.resize(n + 1); // First variable for t, rest for y1, ..., yn
    parsers.resize(n);

    for (size_t i = 0; i < n; ++i) {
        // std::cout << "Initializing parser for function: " << functions[i] << std::endl;

        parsers[i].DefineVar("t", &variables[0]); // Time variable
        for (size_t j = 0; j < n; ++j) {
            parsers[i].DefineVar("y" + std::to_string(j + 1), &variables[j + 1]); // y1, ..., yn
        }

        try {
            parsers[i].SetExpr(functions[i]); // Set the function expression
        } catch (const mu::Parser::exception_type& e) {
            throw std::invalid_argument("Error in parsing function: " + std::string(e.GetMsg()));
        }

        // std::cout << "Parser " << i << " initialized successfully.\n";
    }
}

Eigen::VectorXd FunctionParser::evaluate(const Eigen::VectorXd& input) {
    // std::cout << "FunctionParser::evaluate() called" << std::endl;
    
    if (input.size() != n + 1) {
        throw std::invalid_argument("Input vector size must be " + std::to_string(n + 1) + ".");
    }

    // Assign input to variables
    variables[0] = input[0]; // t
    for (size_t i = 1; i <= n; ++i) {
        variables[i] = input[i]; // y1, y2, ..., yn
    }

    // Debugging: Print out the variables before evaluation
    // std::cout << "Variables for evaluation: ";
    // for (size_t i = 0; i <= n; ++i) {
    //     std::cout << variables[i] << " ";
    // }
    // std::cout << std::endl;

    // Evaluate each function
    Eigen::VectorXd output(n);
    for (size_t i = 0; i < n; ++i) {
        try {
            // Debugging: Print the expression being evaluated
            // std::cout << "Evaluating function " << i + 1 << ": " << f_i[i] << std::endl;

            // Define variables in the parser (this is crucial to bind them correctly)
            parsers[i].DefineVar("t", &variables[0]);  // Make sure t is defined
            for (size_t j = 0; j < n; ++j) {
                parsers[i].DefineVar("y" + std::to_string(j + 1), &variables[j + 1]);  // y1, y2, ..., yn
            }

            output[i] = parsers[i].Eval();

            // std::cout << "Function " << i + 1 << " evaluated to: " << output[i] << std::endl;
        } catch (const mu::Parser::exception_type& e) {
            throw std::runtime_error("Error during evaluation: " + std::string(e.GetMsg()));
        }
    }
    return output;
}

f_TYPE FunctionParser::getFunction() {
    // std::cout << "FunctionParser::getFunction() called" << std::endl;

    // Capture required data by value
    auto parsers_copy = parsers;
    auto n_copy = n;

    return [parsers_copy, n_copy, variables = this->variables](const Eigen::VectorXd& y, double t) mutable {
        // std::cout << "Lambda function called with t: " << t << " and y: ";
        // for (int i = 0; i < n_copy; ++i) {
        //     std::cout << y[i] << " ";
        // }
        // std::cout << std::endl;

        if (y.size() != n_copy) {
            throw std::invalid_argument("Input vector y size must be " + std::to_string(n_copy) + ".");
        }

        // Combine t and y into a single vector
        Eigen::VectorXd input(n_copy + 1);
        input[0] = t; // Assign t to the first position
        input.tail(n_copy) = y; // Assign y1, ..., yn to the rest of the vector

        // Debugging: Print combined input vector
        // std::cout << "Combined input vector for evaluation: ";
        // for (int i = 0; i < input.size(); ++i) {
        //     std::cout << input[i] << " ";
        // }
        // std::cout << std::endl;

        // Update shared variables for the lambda
        variables[0] = t; // Time variable
        for (size_t i = 1; i <= n_copy; ++i) {
            variables[i] = input[i]; // y1, y2, ..., yn
        }

        // Debugging: Print updated variables
        // std::cout << "Variables updated in lambda: ";
        // for (size_t i = 0; i <= n_copy; ++i) {
        //     std::cout << variables[i] << " ";
        // }
        // std::cout << std::endl;

        // Define variables in the parser (this is crucial to bind them correctly)
        for (size_t i = 0; i < n_copy; ++i) {
            parsers_copy[i].DefineVar("t", &variables[0]);  // Ensure t is defined
            for (size_t j = 0; j < n_copy; ++j) {
                parsers_copy[i].DefineVar("y" + std::to_string(j + 1), &variables[j + 1]);  // y1, y2, ..., yn
            }
        }

        // Evaluate the function
        Eigen::VectorXd output(n_copy);
        for (size_t i = 0; i < n_copy; ++i) {
            try {
                // std::cout << "Evaluating function " << i + 1 << ": " << parsers_copy[i].GetExpr() << std::endl;  // Print the expression
                output[i] = parsers_copy[i].Eval();
                // std::cout << "Function " << i + 1 << " evaluated to: " << output[i] << std::endl;
            } catch (const mu::Parser::exception_type& e) {
                throw std::runtime_error("Error during evaluation: " + std::string(e.GetMsg()));
            }
        }
        return output;
    };
}
