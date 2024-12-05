//
// Created by natha on 27/11/2024.
//

#ifndef RUNGEKUTTA_HH
#define RUNGEKUTTA_HH

#include "Explicit.hh"

/**
 * @class RungeKutta
 * @brief A class to solve systems of ordinary differential equations (ODEs) using the Runge-Kutta method with Eigen::VectorXd.
 */
class RungeKutta : public Explicit {
private:
    int order; ///< Order of the Runge-Kutta method.
    Eigen::MatrixXd a; ///< Coefficients for intermediate steps (Butcher tableau).
    Eigen::VectorXd b; ///< Weights for the final summation.
    Eigen::VectorXd c; ///< Nodes (time fractions).

public:
    /**
     * @brief Default constructor for Runge-Kutta methods.
     */
    RungeKutta();

    /**
     * @brief Constructor for predefined Runge-Kutta methods by order.
     * @param order The order of the Runge-Kutta method (1, 2, 3, or 4).
     */
    explicit RungeKutta(int order);

    /**
     * @brief Constructor for user-defined Runge-Kutta coefficients.
     * @param a Coefficients for intermediate steps (Butcher tableau).
     * @param b Weights for the final summation.
     * @param c Nodes (time fractions).
     * @throws std::invalid_argument if has invalid settings
     */
    RungeKutta(const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c);

    /**
     * @brief Sets the coefficients for a predefined Runge-Kutta method.
     * @param order The order of the Runge-Kutta method (1, 2, 3, or 4).
     * @throws std::invalid_argument if the order is unsupported.
     */
    void setOrder(int order);

    void setCoefficients(const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c);
    /**
     * @brief Sets the coefficients for a user-defined Runge-Kutta method.
     * @param Rdr The reader object containing the coefficients.
     */
    void SetConfig(const Reader& Rdr) override;

    Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;  // Vectorized Step
};

#endif // RUNGEKUTTA_HH