/**
 * @file Logger.hh
 * @brief Header file for the `Logger` class.
 *
 * This header defines the `Logger` class, which provides logging functionality
 * with multiple verbosity levels. It allows logging of errors, warnings, informational,
 * and debug messages.
 *
 * Author: janzgraggen
 */

#ifndef LOGGER_HH
#define LOGGER_HH

#include <iostream>
#include <string>

/**
 * @class Logger
 * @brief A class to handle logging with different verbosity levels.
 *
 * The `Logger` class provides methods to log messages at various levels of verbosity,
 * including errors, warnings, informational messages, and debug messages.
 * The verbosity level is set during the construction of the `Logger` object.
 */
class Logger {
public:
    /**
     * @brief Constructor to set the verbosity level.
     *
     * @param verbosity An integer representing the desired logging verbosity level.
     * - Silent: 0
     * - Error: 1
     * - Warning: 2
     * - Info: 3
     * - Debug: 4
     */
    explicit Logger(int verbosity);

    /**
     * @brief Logs an error message.
     *
     * @param message The error message to be logged.
     */
    void error(const std::string& message) const;

    /**
     * @brief Logs a warning message.
     *
     * @param message The warning message to be logged.
     */
    void warning(const std::string& message) const;

    /**
     * @brief Logs an informational message.
     *
     * @param message The informational message to be logged.
     */
    void info(const std::string& message) const;

    /**
     * @brief Logs a debug message.
     *
     * @param message The debug message to be logged.
     */
    void debug(const std::string& message) const;

private:
    int verbosity;  //!< Current verbosity level of logging.
    static const int Silent = 0;  //!< Logging level for silent mode.
    static const int Error = 1;   //!< Logging level for errors only.
    static const int Warning = 2; //!< Logging level for warnings.
    static const int Info = 3;    //!< Logging level for informational messages.
    static const int Debug = 4;   //!< Logging level for debug messages.
};

#endif // LOGGER_HH