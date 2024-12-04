#include "Reader.hh"


// Constructor to load the YAML file
Reader::Reader(const str& filename) {
    config = YAML::LoadFile(filename);
}

// General configuration method to get the solver type
str Reader::getSolverType() const {
    return config["solver_type"].as<str>();
}

// Get OdeSolver settings
Reader::OdeSettings Reader::getOdeSettings() const {
    OdeSettings odeSolverSettings;
    auto odeSolverNode = config["OdeSolver"];

    // Read values directly from the YAML node
    odeSolverSettings.step_size = odeSolverNode["step_size"].as<double>();
    odeSolverSettings.initial_time = odeSolverNode["initial_time"].as<double>();
    odeSolverSettings.final_time = odeSolverNode["final_time"].as<double>();

    // Read the initial value as an Eigen::VectorXd
    auto initial_value = odeSolverNode["initial_value"].as<std::vector<double>>();
    odeSolverSettings.initial_value = Eigen::VectorXd::Map(initial_value.data(), initial_value.size());

    return odeSolverSettings;
}

// Get Explicit solver settings
Reader::ExplicitSettings Reader::getExplicitSettings() const {
    ExplicitSettings explSet;
    auto explicitNode = config["Explicit"];
    explSet.method = explicitNode["method"].as<str>();
    if (explSet.method == "ForwardEuler") {

    // Check if RungeKutta method is selected
    } else if (explSet.method == "RungeKutta") {

        // Check if an 'order' int is provided
        if (explicitNode["RungeKutta"]["order"].IsScalar()) {
            explSet.RungeKutta_order = explicitNode["RungeKutta"]["order"].as<int>();
        }
        // If 'order' is not provided, check if all 'coefficients' are available
        else if (explicitNode["RungeKutta"]["coefficients"]["a"].IsSequence()
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
            throw std::runtime_error("Invalid RungeKutta settings: Either 'order' or 'coefficients' must be provided.");
        }
    } else if (explSet.method == "AdamsBashforth") {
        if (explicitNode["AdamsBashforth"]["max_order"].IsScalar()) {
            explSet.AdamsBashforth_max_order = explicitNode["AdamsBashforth"]["max_order"].as<int>();
        } else if (explicitNode["AdamsBashforth"]["coefficients_vector"].IsSequence()) {
            auto v = explicitNode["AdamsBashforth"]["coefficients_vector"].as<std::vector<double>>();
            explSet.AdamsBashforth_coefficients_vector = Eigen::VectorXd::Map(v.data(), v.size());
        } else {
            throw std::runtime_error("Invalid AdamsBashforth settings: Either 'maxOrder' or 'coefficients_vector' must be provided.");
        }
    } else {
        throw std::runtime_error("Invalid solver method: " + explSet.method);
    }

    return explSet;
}

// Get Implicit solver settings
Reader::ImplicitSettings Reader::getImplicitSettings() const {
    ImplicitSettings implSet;
    auto implicitNode = config["Implicit"];
    if (implicitNode["method"].IsScalar()
    && implicitNode["rhs_is_linear"].IsScalar()
    && implicitNode["linear_system_solver"].IsScalar()) {
        implSet.method = implicitNode["method"].as<str>();
        implSet.rhs_is_linear = implicitNode["rhs_is_linear"].as<bool>();
        implSet.linear_system_solver = implicitNode["linear_system_solver"].as<str>();
    }
    if (implSet.rhs_is_linear) {
        //auto rhsNode = implicitNode["rhs_system"];
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
            throw std::runtime_error("Invalid rhs system settings: A is not provided.");
        }
        if (implicitNode["rhs_system"]["b"].IsSequence()) {
            auto b = implicitNode["rhs_system"]["b"].as<std::vector<double>>();
            implSet.rhs_system->b = Eigen::VectorXd::Map(b.data(), b.size());
        }
    } else {
        // Handle non-linear cases
        if (implicitNode["tolerance"].IsScalar()
        && implicitNode["max_iterations"].IsScalar()
        && implicitNode["root_finder"].IsDefined()
        && implicitNode["dx"].IsScalar()) {
            implSet.tolerance = implicitNode["tolerance"].as<double>();
            implSet.max_iterations = implicitNode["max_iterations"].as<int>();
            implSet.dx = implicitNode["dx"].as<double>();
            implSet.root_finder = implicitNode["root_finder"].as<str>();
        } else {
            throw std::runtime_error("Invalid implicit nonlinear settings: missing entries.");
        }
    }

    return implSet;
}