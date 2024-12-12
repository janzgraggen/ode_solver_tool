/**
 * @file Reader.cc
 * @brief Implements the `Reader` class that loads configuration from a YAML file for ODE solvers.
 *
 * This source file contains the implementation of the `Reader` class, which loads ODE solver configurations
 * from a YAML file. It parses solver types, function strings, and solver parameters while supporting
 * explicit and implicit solver configurations. The class integrates seamlessly with the `FunctionParser`
 * and YAML parsing library to provide a structured way to read and apply solver configurations.
 *
 */

#include "Reader.hh"
#include "FunctionParser.hh"
#include "../Logger/Logger.hh"

/**
 * @brief Constructs a `Reader` object and loads the specified YAML configuration file.
 *
 * Loads the YAML configuration file provided by `filename`. If the file cannot be opened or parsed,
 * a `std::runtime_error` is thrown.
 * @param logger_ Reference to the `Logger` object.
 * @param filename The path to the YAML configuration file.
 * @throws std::runtime_error If the file cannot be opened or parsed.
 */
Reader::Reader(Logger& logger_,const str& filename) : logger(&logger_) {
    config = YAML::LoadFile(filename);
}

/**
 * @brief Retrieves the solver type specified in the configuration.
 *
 * Reads the `solver_type` entry from the YAML configuration file.
 *
 * @return A string indicating the solver type (e.g., "Implicit" or "Explicit").
 */
str Reader::getSolverType() const {
    return config["solver_type"].as<str>();
}

/**
 * @brief Retrieves the output filename specified in the configuration.
 *
 * Reads the `output_file` entry from the YAML configuration file.
 *
 * @return A string representing the output filename.
 */
str Reader::getOutputFileName() const {
    return config["output_file"].as<str>();
}

/**
 * @brief Retrieves the getDimension of the system from the configuration.
 *
 * Reads the `Dim` entry from the YAML configuration file, representing the number of state variables.
 *
 * @return An integer representing the getDimension of the system.
 */
int Reader::getDim() const {
    return config["Dim"].as<int>();
}

/**
 * @brief Retrieves the list of function strings for the system from the configuration.
 *
 * Constructs a list of function strings stored under the `Function` section in the YAML file.
 *
 * @return A `strList` containing function strings corresponding to the getDimension of the system.
 */
strList Reader::getFunctionStringlist() const {
    strList fct_strings;
    fct_strings.resize(getDim());

    for (int i = 0; i < getDim(); i++) {
        std::string key = "f" + std::to_string(i + 1);
        fct_strings[i] = config["Function"][key].as<str>();
    }
    return fct_strings;
}

/**
 * @brief Returns a callable function that evaluates the stored functions dynamically.
 *
 * Constructs and returns a callable `f_TYPE` that uses the `FunctionParser` to bind and evaluate
 * functions dynamically based on the configuration's state and time input.
 *
 * @return A callable `f_TYPE` lambda function that evaluates the functions given a state vector `y` and time `t`.
 */
f_TYPE Reader::getFunction() const {
    return FunctionParser(getDim(), getFunctionStringlist()).getFunction();
}

/**
 * @brief Retrieves the verbosity level from the configuration.
 *
 * Reads the `verbosity_level` entry in the YAML configuration file.
 *
 * @return An integer representing the verbosity level.
 */
int Reader::getVerbosity() const {
    return config["verbosity_level"].as<int>();
}


/**
 * @brief Sets the logger verbosity level based on the configuration.
 */
void Reader::setLoggerVerbosity() {
    logger->setVerbosity(getVerbosity());
}

/**
 * @brief Retrieves the general settings for the ODE solver from the configuration.
 *
 * Parses the `OdeSolver` section of the YAML file and constructs an `OdeSettings` structure.
 *
 * @return An `OdeSettings` structure containing solver parameters like step size and initial values.
 */
OdeSettings Reader::getOdeSettings() const {
    OdeSettings odeSolverSettings;
    auto odeSolverNode = config["OdeSolver"];

    odeSolverSettings.step_size = odeSolverNode["step_size"].as<double>();
    odeSolverSettings.initial_time = odeSolverNode["initial_time"].as<double>();
    odeSolverSettings.final_time = odeSolverNode["final_time"].as<double>();

    auto initial_value = odeSolverNode["initial_value"].as<std::vector<double>>();
    odeSolverSettings.initial_value = Eigen::VectorXd::Map(initial_value.data(), initial_value.size());

    return odeSolverSettings;
}

/**
 * @brief Retrieves the settings for explicit solvers from the configuration.
 *
 * Parses the `Explicit` section of the YAML configuration and sets up the solver parameters
 * for explicit solvers, including support for different methods like Forward Euler, Runge-Kutta,
 * and Adams-Bashforth.
 *
 * @return An `ExplicitSettings` structure containing explicit solver parameters.
 * @throws std::runtime_error If the configuration for an explicit solver is invalid.
 */
ExplicitSettings Reader::getExplicitSettings() const {
    ExplicitSettings explSet;
    auto explicitNode = config["Explicit"];
    explSet.method = explicitNode["method"].as<str>();

    if (explSet.method == "ForwardEuler") {
        // Currently no specific configuration for Forward Euler method.
    } else if (explSet.method == "RungeKutta") {
        if (explicitNode["RungeKutta"]["order"].IsScalar()) {
            explSet.RungeKutta_order = explicitNode["RungeKutta"]["order"].as<int>();
        } else if (explicitNode["RungeKutta"]["coefficients"]["a"].IsSequence()
                && explicitNode["RungeKutta"]["coefficients"]["b"].IsSequence()
                && explicitNode["RungeKutta"]["coefficients"]["c"].IsSequence()) {
            auto rkCoefsNode = explicitNode["RungeKutta"]["coefficients"];

            auto a = rkCoefsNode["a"].as<std::vector<std::vector<double>>>();
            Eigen::MatrixXd a_matrix(a.size(), a[0].size());

            for (size_t i = 0; i < a.size(); ++i) {
                a_matrix.row(i) = Eigen::VectorXd::Map(a[i].data(), a[i].size());
            }
            explSet.RungeKutta_coefficients_a = a_matrix;

            auto b = rkCoefsNode["b"].as<std::vector<double>>();
            explSet.RungeKutta_coefficients_b = Eigen::VectorXd::Map(b.data(), b.size());

            auto c = rkCoefsNode["c"].as<std::vector<double>>();
            explSet.RungeKutta_coefficients_c = Eigen::VectorXd::Map(c.data(), c.size());
        } else {
            logger->error("{in Reader::getExplicitSettings()} Invalid RungeKutta settings: Either 'order' or 'coefficients' must be provided.");
        }
    } else if (explSet.method == "AdamsBashforth") {
        if (explicitNode["AdamsBashforth"]["max_order"].IsScalar()) {
            explSet.AdamsBashforth_max_order = explicitNode["AdamsBashforth"]["max_order"].as<int>();
        } else if (explicitNode["AdamsBashforth"]["coefficients_vector"].IsSequence()) {
            auto v = explicitNode["AdamsBashforth"]["coefficients_vector"].as<std::vector<double>>();
            explSet.AdamsBashforth_coefficients_vector = Eigen::VectorXd::Map(v.data(), v.size());
        } else {
            logger->error("{in Reader::getExplicitSettings()} Invalid AdamsBashforth settings: Either 'maxOrder' or 'coefficients_vector' must be provided.");

        }
    } else {
        logger->error("{in Reader::getExplicitSettings()} Invalid solver method: " + explSet.method);
    }

    return explSet;
}

/**
 * @brief Retrieves the settings for implicit solvers from the configuration.
 *
 * Parses the `Implicit` section of the YAML configuration, constructing an `ImplicitSettings` structure
 * with support for both linear and nonlinear solver configurations.
 *
 * @return An `ImplicitSettings` structure containing implicit solver parameters.
 * @throws std::runtime_error If the configuration for an implicit solver is invalid.
 */
ImplicitSettings Reader::getImplicitSettings() const {
    ImplicitSettings implSet = {};
    auto implicitNode = config["Implicit"];

    if (implicitNode["method"].IsScalar()
        && implicitNode["rhs_is_linear"].IsScalar()
        && implicitNode["linear_system_solver"].IsScalar()) {
        implSet.method = implicitNode["method"].as<str>();
        implSet.rhs_is_linear = implicitNode["rhs_is_linear"].as<bool>();
        implSet.linear_system_solver = implicitNode["linear_system_solver"].as<str>();
    }

    if (implSet.rhs_is_linear) {
        if (implicitNode["rhs_system"]["A"].IsSequence()) {
            auto A_matrix = implicitNode["rhs_system"]["A"].as<std::vector<std::vector<double>>>();
            Eigen::MatrixXd A(A_matrix.size(), A_matrix[0].size());

            for (size_t i = 0; i < A_matrix.size(); ++i) {
                for (size_t j = 0; j < A_matrix[i].size(); ++j) {
                    A(i, j) = A_matrix[i][j];
                }
            }
            LinearSystem SysToSet(A, Eigen::VectorXd::Zero(A.rows()));
            implSet.rhs_system = SysToSet;

        } else {
            logger->error("{in Reader::getImplicitSettings()} Invalid rhs system settings: A is not provided.");
        }

        if (implicitNode["rhs_system"]["b"].IsSequence()) {
            auto b = implicitNode["rhs_system"]["b"].as<std::vector<double>>();
            implSet.rhs_system->b = Eigen::VectorXd::Map(b.data(), b.size());
        }
    } else {
        if (implicitNode["tolerance"].IsScalar()
            && implicitNode["max_iterations"].IsScalar()
            && implicitNode["root_finder"].IsDefined()
            && implicitNode["dx"].IsScalar()) {
            implSet.tolerance = implicitNode["tolerance"].as<double>();
            implSet.max_iterations = implicitNode["max_iterations"].as<int>();
            implSet.dx = implicitNode["dx"].as<double>();
            implSet.root_finder = implicitNode["root_finder"].as<str>();
        } else {
            logger->error("{in Reader::getImplicitSettings()} Invalid nonlinear settings: missing entries.");
        }
    }

    return implSet;
}
