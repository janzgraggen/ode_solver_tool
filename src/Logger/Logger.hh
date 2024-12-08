#ifndef LOGGER_HH
#define LOGGER_HH

#include <iostream>
#include <string>

class Logger {
public:
    // Constructor to set the verbosity level using an integer
    explicit Logger(int verbosity);

    // Method to log error messages
    void error(const std::string& message) const;

    // Method to log warning messages
    void warning(const std::string& message) const;

    // Method to log info messages
    void info(const std::string& message) const;

    // Method to log debug messages
    void debug(const std::string& message) const;

private:
    int verbosity;  // Current verbosity level as an integer
    static const int Silent = 0;
    static const int Error = 1;
    static const int Warning = 2;
    static const int Info = 3;
    static const int Debug = 4;
};

#endif // LOGGER_HH
