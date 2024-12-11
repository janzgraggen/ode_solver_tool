/**
* @file main.cc
 * @brief Main application entry point for running the ODE solver.
 *
 * This program initializes the `Runner` class with a configuration file containing
 * system parameters and uses it to solve an Ordinary Differential Equation (ODE).
 * The `Runner` reads the configuration, runs the solver, and outputs the final solution
 * to the console, displaying the state variables in a structured format.
 *
 * @author natha
 * @date 25/11/2024
 */

#include <iostream>
#include "Reader/Reader.hh"
#include "Runner/Runner.hh"

/**
 * @brief Main function to execute the ODE solver process.
 *
 * This function:
 * - Initializes a `Runner` instance with the path to the configuration file.
 * - Calls the `run()` method on the `Runner` object to solve the system of ODEs.
 * - Outputs the computed solution vector to the console.
 *
 * @return Exit status of the program (0 for successful execution).
 */
int main() {
    try {
    // Create a Runner instance using the configuration file for system parameters
    Logger* loggerPtr = new Logger(0);
    Runner runner(*loggerPtr, "../config/main/ODE_config.yaml");

    // Run the solver and get the final computed solution
    const Eigen::VectorXd finalSolution = runner.run();

    // Output the final solution to the console
    std::cout << "Final solution:"  << std::endl;
    for (int i = 0; i < finalSolution.size(); ++i) {
        // Print each state variable y1, y2, ..., yn
        std::cout << "  y" << i + 1 << ": " << finalSolution(i) << std::endl;
    }

    return 0;

    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
        return 1;
    }
}