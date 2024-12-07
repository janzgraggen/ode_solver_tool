#ifndef LOGGER_HH
#define LOGGER_HH

#include <iostream>
#include <string>

// Enum to define different verbosity levels
enum class Verbosity {
    Silent = 0,  // No output
    Error,       // Error messages only
    Warning,     // Error and warning messages
    Info,        // Error, warning, and info messages
    Debug        // Error, warning, info, and debug messages
};

class Logger {
public:
    // Constructor to set the verbosity level
    explicit Logger(Verbosity verbosity);

    // Method to log error messages
    void error(const std::string& message) const;

    // Method to log warning messages
    void warning(const std::string& message) const;

    // Method to log info messages
    void info(const std::string& message) const;

    // Method to log debug messages
    void debug(const std::string& message) const;

private:
    Verbosity verbosity;  // Current verbosity level
};

#endif // LOGGER_HH
