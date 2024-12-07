#include "Logger.hh"

// Constructor to initialize the verbosity level
Logger::Logger(Verbosity verbosity) : verbosity(verbosity) {}

// Log error messages, only if verbosity level is >= Error
void Logger::error(const std::string& message) const {
    if (verbosity >= Verbosity::Error) {
        std::cerr << "[ERROR]: " << message << std::endl;
    }
}

// Log warning messages, only if verbosity level is >= Warning
void Logger::warning(const std::string& message) const {
    if (verbosity >= Verbosity::Warning) {
        std::cout << "[WARNING]: " << message << std::endl;
    }
}

// Log info messages, only if verbosity level is >= Info
void Logger::info(const std::string& message) const {
    if (verbosity >= Verbosity::Info) {
        std::cout << "[INFO]: " << message << std::endl;
    }
}

// Log debug messages, only if verbosity level is >= Debug
void Logger::debug(const std::string& message) const {
    if (verbosity >= Verbosity::Debug) {
        std::cout << "[DEBUG]: " << message << std::endl;
    }
}
