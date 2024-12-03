#ifndef READER_HH
#define READER_HH

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <optional>
#include "../LinSysStruct.hh"

using str = std::string;

class Reader {
public:
    // Constructor to load the YAML file
    Reader(const str& filename);

    // Method to get the solver type (Implicit or Explicit)
    str getSolverType() const;

    // OdeSolver settings structure
    struct OdeSettings {
        double step_size;
        double initial_time;
        double final_time;
        Eigen::VectorXd initial_value;
    };

    // Explicit solver settings structure
    struct ExplicitSettings {
        str method;  // RungeKutta, AdamsBashforth, etc.
        std::optional<int> RungeKutta_order;
        std::optional<Eigen::MatrixXd> RungeKutta_coefficients_a;
        std::optional<Eigen::VectorXd> RungeKutta_coefficients_b;
        std::optional<Eigen::VectorXd> RungeKutta_coefficients_c;

        std::optional<int> AdamsBashforth_max_order;
        std::optional<Eigen::VectorXd> AdamsBashforth_coefficients_vector;
    };

    // Implicit solver settings structure
    struct ImplicitSettings {
        str method;  // BackwardEuler, CrankNicolson
        bool rhs_is_linear;
        std::optional<LinearSystem> rhs_system;

        // Nonlinear settings
        std::optional<double> tolerance;
        std::optional<int> max_iterations;
        std::optional<str> root_finding_method;
        std::optional<double> dx;
        std::optional<str> linear_system_solver;
    };

    // Methods to retrieve the settings
    OdeSettings getOdeSettings() const;
    ExplicitSettings getExplicitSettings() const;
    ImplicitSettings getImplicitSettings() const;

private:
    // YAML configuration object
    YAML::Node config;
};

#endif // READER_HH
