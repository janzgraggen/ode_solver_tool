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

/**
 * @brief Constructs a `Logger` instance and sets the logging verbosity level.
 *
 * The constructor initializes the logging instance with a specified verbosity level.
 *
 * @param verbosity An integer representing the desired logging verbosity.
 *                  Higher values correspond to more verbose logging levels.
 */
Logger::Logger(int verbosity) : verbosity(verbosity) {}

/**
 * @brief Logs error messages.
 *
 * Logs error messages only if the verbosity level is greater than or equal to the `Error` level.
 *
 * @param message The error message to be logged.
 */
void Logger::error(const std::string& message) const {
    if (verbosity >= err) {
        std::string  error_msg = "[ERROR]: " + message;
        //std::cerr << error_msg << std::endl;
        throw std::runtime_error(error_msg);
    }
}

/**
 * @brief Logs warning messages.
 *
 * Logs warning messages only if the verbosity level is greater than or equal to the `Warning` level.
 *
 * @param message The warning message to be logged.
 */
void Logger::warning(const std::string& message) const {
    if (verbosity >= warn) {
        std::cout << "[WARNING]: " << message << std::endl;
    }
}

/**
 * @brief Logs informational messages.
 *
 * Logs informational messages only if the verbosity level is greater than or equal to the `Info` level.
 *
 * @param message The informational message to be logged.
 */
void Logger::info(const std::string& message) const {
    if (verbosity >= inf) {
        std::cout << "[INFO]: " << message << std::endl;
    }
}

/**
 * @brief Logs debug messages.
 *
 * Logs debug messages only if the verbosity level is greater than or equal to the `Debug` level.
 *
 * @param message The debug message to be logged.
 */
void Logger::debug(const std::string& message) const {
    if (verbosity >= debg) {
        std::cout << "[DEBUG]: " << message << std::endl;
    }
}

void Logger::setVerbosity(int verbosity_) {
    this->verbosity = verbosity_;
}