#include "Reader.hh"

// Constructor to load the YAML file
Reader::Reader(const str& filename) {
    // Load the YAML configuration file
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

    if (explSet.method == "RungeKutta") {
        explSet.RungeKutta_order = explicitNode["RungeKutta"]["order"].as<int>();

        // Read coefficients as Eigen types
        auto a = explicitNode["RungeKutta"]["coefficients"]["a"].as<std::vector<std::vector<double>>>();
        Eigen::MatrixXd a_matrix(a.size(), a[0].size());
        for (size_t i = 0; i < a.size(); ++i) {
            a_matrix.row(i) = Eigen::VectorXd::Map(a[i].data(), a[i].size());
        }
        explSet.RungeKutta_coefficients_a = a_matrix;

        explSet.RungeKutta_coefficients_b = Eigen::VectorXd::Map(explicitNode["RungeKutta"]["coefficients"]["b"].as<std::vector<double>>().data(), a[0].size());
        explSet.RungeKutta_coefficients_c = Eigen::VectorXd::Map(explicitNode["RungeKutta"]["coefficients"]["c"].as<std::vector<double>>().data(), a[0].size());

    } else if (explSet.method == "AdamsBashforth") {
        explSet.AdamsBashforth_max_order = explicitNode["AdamsBashforth"]["max_order"].as<int>();
        explSet.AdamsBashforth_coefficients_vector = Eigen::VectorXd::Map(explicitNode["AdamsBashforth"]["coefficients_vector"].as<std::vector<double>>().data(),
                                                                           explicitNode["AdamsBashforth"]["coefficients_vector"].as<std::vector<double>>().size());
    }

    return explSet;
}

// Get Implicit solver settings
Reader::ImplicitSettings Reader::getImplicitSettings() const {
    ImplicitSettings implSet;
    auto implicitNode = config["Implicit"];

    implSet.method = implicitNode["method"].as<str>();
    implSet.rhs_is_linear = implicitNode["rhs_is_linear"].as<bool>();

    if (implSet.rhs_is_linear) {
        // Read matrices A and b if available
        if (implicitNode["rhs_system"]["A"] || implicitNode["rhs_system"]["b"]){
            LinearSystem rhs_sys_struct;
        
        
            if (implicitNode["rhs_system"]["A"]) {
                auto A_matrix = implicitNode["rhs_system"]["A"].as<std::vector<std::vector<double>>>();
                Eigen::MatrixXd A(A_matrix.size(), A_matrix[0].size());
                for (size_t i = 0; i < A_matrix.size(); ++i) {
                    A.row(i) = Eigen::VectorXd::Map(A_matrix[i].data(), A_matrix[i].size());
                }
                rhs_sys_struct.A = A;
            }

            if (implicitNode["rhs_system"]["b"]) {
                rhs_sys_struct.b = Eigen::VectorXd::Map(implicitNode["rhs_system"]["b"].as<std::vector<double>>().data(), rhs_sys_struct.A.cols());
            }

            implSet.rhs_system = rhs_sys_struct;    
        }
    } else {
        // Read settings for nonlinear case if available
        if (implicitNode["tolerance"]) implSet.tolerance = implicitNode["tolerance"].as<double>();
        if (implicitNode["max_iterations"]) implSet.max_iterations = implicitNode["max_iterations"].as<int>();
        if (implicitNode["root_finding_method"]) implSet.root_finding_method = implicitNode["root_finding_method"].as<str>();
        if (implicitNode["dx"]) implSet.dx = implicitNode["dx"].as<double>();
        if (implicitNode["linear_system_solver"]) implSet.linear_system_solver = implicitNode["linear_system_solver"].as<str>();
    }

    return implSet;
}
