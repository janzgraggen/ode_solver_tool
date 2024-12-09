/**
* @file Logger.cc
 * @brief Implementation of the `Logger` class methods.
 *
 * This source file contains the method definitions for the `Logger` class,
 * which provides logging functionality with multiple verbosity levels.
 * The verbosity levels control the output of error, warning, informational, and debug messages.
 *
 * Author: janzgraggen
 * Date: 27/11/2024
 */

#include "Logger.hh"

// Constructor to initialize the logging verbosity level.
// @param verbosity An integer representing the desired logging verbosity.
Logger::Logger(int verbosity) : verbosity(verbosity) {}

// Log error messages, only if the verbosity level is >= Error.
// @param message The error message to be logged.
void Logger::error(const std::string& message) const {
    if (verbosity >= Error) {
        std::cerr << "[ERROR]: " << message << std::endl;
    }
}

// Log warning messages, only if the verbosity level is >= Warning.
// @param message The warning message to be logged.
void Logger::warning(const std::string& message) const {
    if (verbosity >= Warning) {
        std::cout << "[WARNING]: " << message << std::endl;
    }
}

// Log informational messages, only if the verbosity level is >= Info.
// @param message The informational message to be logged.
void Logger::info(const std::string& message) const {
    if (verbosity >= Info) {
        std::cout << "[INFO]: " << message << std::endl;
    }
}

// Log debug messages, only if the verbosity level is >= Debug.
// @param message The debug message to be logged.
void Logger::debug(const std::string& message) const {
    if (verbosity >= Debug) {
        std::cout << "[DEBUG]: " << message << std::endl;
    }
}
