/**
 * @file RungeKutta.cc
 * @brief Implementation of the RungeKutta class for solving ODEs using explicit Runge-Kutta methods.
 *
 * This source file contains the implementation of the RungeKutta class, which extends the Explicit class
 * to provide various explicit Runge-Kutta methods for solving ordinary differential equations (ODEs).
 *
 * The class supports predefined methods by order (e.g., Euler, midpoint, classical Runge-Kutta) and
 * user-defined coefficients. It also integrates seamlessly with the Reader class for configuration.
 *
 * Author: natha
 * Date: 27/11/2024
 */

#include "RungeKutta.hh"

/**
 * @brief Default constructor for Runge-Kutta methods.
 *
 * Initializes a Runge-Kutta solver with default settings, inheriting from the Explicit class.
 *
 * @param logger_ A reference to the Logger object for logging messages.
 */
RungeKutta::RungeKutta(Logger& logger_) : Explicit(logger_) {}

/**
 * @brief Constructor for predefined Runge-Kutta methods by order.
 *
 * Initializes a Runge-Kutta solver with coefficients based on the specified method order.
 *
 * @param logger_ A reference to the Logger object for logging messages.
 * @param order The order of the predefined Runge-Kutta method (e.g., 1 for Euler, 4 for classical Runge-Kutta).
 */
RungeKutta::RungeKutta(Logger& logger_, const int order) : Explicit(logger_), order(order) {
    setOrder(order);
}

/**
 * @brief Constructor for user-defined Runge-Kutta coefficients.
 *
 * Allows the user to specify custom coefficients for the Runge-Kutta method.
 *
 * @param logger_ A reference to the Logger object for logging messages.
 * @param a The matrix of stage coefficients (Butcher tableau).
 * @param b The vector of weights for the linear combination of stages.
 * @param c The vector of nodes (time points) for the stages.
 */
RungeKutta::RungeKutta(Logger& logger_, const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c)
    : Explicit(logger_), a(a), b(b), c(c), order(static_cast<int>(b.size())) {}

/**
 * @brief Sets the coefficients for a predefined Runge-Kutta method.
 *
 * Configures the Runge-Kutta solver with standard coefficients based on the specified method order.
 * Throws an exception if the order is unsupported.
 *
 * @param order The desired order of the Runge-Kutta method.
 * @throw std::invalid_argument If the specified order is not supported.
 */
void RungeKutta::setOrder(const int order) {
    if (order == 1) {
        // Euler's method
        a = Eigen::MatrixXd::Zero(1, 1);
        b = Eigen::VectorXd::Ones(1);
        c = Eigen::VectorXd::Zero(1);
    } else if (order == 2) {
        // Midpoint method
        a = Eigen::MatrixXd::Zero(2, 2);
        a(1, 0) = 0.5;

        b = Eigen::VectorXd(2);
        b << 0, 1;

        c = Eigen::VectorXd(2);
        c << 0, 0.5;
    } else if (order == 3) {
        // Order 3
        a = Eigen::MatrixXd::Zero(3, 3);
        a(1, 0) = 0.5;
        a(2, 0) = -1;
        a(2, 1) = 2;

        b = Eigen::VectorXd(3);
        b << 1.0 / 6, 2.0 / 3, 1.0 / 6;

        c = Eigen::VectorXd(3);
        c << 0, 0.5, 1;
    } else if (order == 4) {
        // Classical Runge-Kutta
        a = Eigen::MatrixXd::Zero(4, 4);
        a(1, 0) = 0.5;
        a(2, 1) = 0.5;
        a(3, 2) = 1;

        b = Eigen::VectorXd(4);
        b << 1.0 / 6, 1.0 / 3, 1.0 / 3, 1.0 / 6;

        c = Eigen::VectorXd(4);
        c << 0, 0.5, 0.5, 1;
    } else {
        logger->error("{in RungeKutta::setOrder()} Unsupported order: " + std::to_string(order));
    }
    this->order = order;
}

/**
 * @brief Sets custom coefficients for the Runge-Kutta method.
 *
 * Updates the coefficients with user-defined values and sets the method order accordingly.
 *
 * @param a The matrix of stage coefficients (Butcher tableau).
 * @param b The vector of weights for the linear combination of stages.
 * @param c The vector of nodes (time points) for the stages.
 */
void RungeKutta::setCoefficients(const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c) {
    this->a = a;
    this->b = b;
    this->c = c;
    this->order = static_cast<int>(b.size());
}

/**
 * @brief Configures the Runge-Kutta solver using a Reader object.
 *
 * Loads the solver configuration, including method order or custom coefficients, from the Reader.
 *
 * @param Rdr The Reader object containing the configuration settings.
 * @throw std::invalid_argument If the configuration is invalid or incomplete.
 */
void RungeKutta::setConfig(const Reader& Rdr) {
    setGlobalConfig(Rdr); // Call the base class method
    ExplicitSettings settings = Rdr.getExplicitSettings();
    if (settings.RungeKutta_order.has_value()) {
        setOrder(settings.RungeKutta_order.value());
    } else if (settings.RungeKutta_coefficients_a.has_value() &&
               settings.RungeKutta_coefficients_b.has_value() &&
               settings.RungeKutta_coefficients_c.has_value()) {
        setCoefficients(
            settings.RungeKutta_coefficients_a.value(),
            settings.RungeKutta_coefficients_b.value(),
            settings.RungeKutta_coefficients_c.value());
    } else {
        logger->error("{in RungeKutta::setConfig()} Missing RungeKutta settings");
    }
}

/**
 * @brief Computes a single step of the Runge-Kutta method.
 *
 * Advances the solution by one step using the Runge-Kutta integration formula.
 *
 * @param y The current state vector \( y_n \).
 * @param t The current time \( t_n \).
 * @return The updated state vector \( y_{n+1} \).
 */
Eigen::VectorXd RungeKutta::calcStep(const Eigen::VectorXd& y, double t) {
    const int n = y.size(); // Dimension of the state vector
    std::vector<Eigen::VectorXd> k(order, Eigen::VectorXd::Zero(n)); // Stage derivatives

    for (int i = 0; i < order; ++i) {
        Eigen::VectorXd sum = Eigen::VectorXd::Zero(n);
        for (int j = 0; j < i; ++j) {
            sum += a(i, j) * k[j];
        }
        k[i] = getRightHandSide()(y + getStepSize() * sum, t + c[i] * getStepSize());
    }

    Eigen::VectorXd increment = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < order; ++i) {
        increment += b[i] * k[i];
    }

    return y + getStepSize() * increment;
}