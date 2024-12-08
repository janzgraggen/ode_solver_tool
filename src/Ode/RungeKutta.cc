//
// Created by natha on 27/11/2024.
//

#include "RungeKutta.hh"


/**
 * @brief Default constructor for Runge-Kutta methods.
 */
RungeKutta::RungeKutta(Logger& logger_) : Explicit(logger_) {};
 
/**
 * @brief Constructor for predefined Runge-Kutta methods by order.
 */
RungeKutta::RungeKutta(Logger& logger_,const int order) : Explicit(logger_), order(order) {
    setOrder(order);
}

/**
 * @brief Constructor for user-defined Runge-Kutta coefficients.
 */
RungeKutta::RungeKutta(Logger& logger_,const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c)
    : Explicit(logger_),a(a), b(b), c(c), order(static_cast<int>(b.size())) {}

/**
 * @brief Sets the coefficients for a predefined Runge-Kutta method.
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
        throw std::invalid_argument("Unsupported order!");
    }
    this->order = order;
}

void RungeKutta::setCoefficients(const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c) {
    this->a = a;
    this->b = b;
    this->c = c;
    this->order = static_cast<int>(b.size());
}

void RungeKutta::SetConfig(const Reader& Rdr) {
    SetGlobalConfig(Rdr); // Call the base class method

    if (Rdr.getExplicitSettings().RungeKutta_order.has_value()) {
        setOrder(Rdr.getExplicitSettings().RungeKutta_order.value());
    } else if (Rdr.getExplicitSettings().RungeKutta_coefficients_a.has_value() &&
               Rdr.getExplicitSettings().RungeKutta_coefficients_b.has_value() &&
               Rdr.getExplicitSettings().RungeKutta_coefficients_c.has_value()) {
        setCoefficients(
            Rdr.getExplicitSettings().RungeKutta_coefficients_a.value(),
            Rdr.getExplicitSettings().RungeKutta_coefficients_b.value(),
            Rdr.getExplicitSettings().RungeKutta_coefficients_c.value());
    } else {
        throw std::invalid_argument("Invalid RungeKutta settings");
    }
}

/**
 * @brief Computes a single step of the Runge-Kutta method.
 * @param y The current state vector \( y_n \).
 * @param t The current time \( t_n \).
 * @return The updated state vector \( y_{n+1} \).
 */
Eigen::VectorXd RungeKutta::Step(const Eigen::VectorXd& y, double t) {
    const int n = y.size(); // Dimension of the state vector
    std::vector<Eigen::VectorXd> k(order, Eigen::VectorXd::Zero(n)); // Stage derivatives

    for (int i = 0; i < order; ++i) {
        Eigen::VectorXd sum = Eigen::VectorXd::Zero(n);
        for (int j = 0; j < i; ++j) {
            sum += a(i, j) * k[j];
        }
        k[i] = GetRightHandSide()(y + GetStepSize() * sum, t + c[i] * GetStepSize());
    }

    Eigen::VectorXd increment = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < order; ++i) {
        increment += b[i] * k[i];
    }

    return y + GetStepSize() * increment;
}