/**
 * @file RungeKutta.hh
 * @brief A class to solve systems of ordinary differential equations (ODEs) using the Runge-Kutta method with Eigen::VectorXd.
 *
 * @author natha
 * @date 27/11/2024
 */

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
    explicit RungeKutta(Logger& logger_);

    /**
     * @brief Constructor for predefined Runge-Kutta methods by order.
     * @param logger_ The logger instance to handle logging.
     * @param order The order of the Runge-Kutta method (1, 2, 3, or 4).
     */
    explicit RungeKutta(Logger& logger_, int order);

    /**
     * @brief Constructor for user-defined Runge-Kutta coefficients.
     * @param logger_ The logger instance to handle logging.
     * @param a Coefficients for intermediate steps (Butcher tableau).
     * @param b Weights for the final summation.
     * @param c Nodes (time fractions).
     * @throws std::invalid_argument if the provided coefficients have invalid settings.
     */
    RungeKutta(Logger& logger_, const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c);

    /**
     * @brief Sets the coefficients for a predefined Runge-Kutta method.
     * @param order The order of the Runge-Kutta method (1, 2, 3, or 4).
     * @throws std::invalid_argument if the provided order is unsupported.
     */
    void setOrder(int order);

    /**
     * @brief Sets the coefficients for a user-defined Runge-Kutta method.
     * @param a Coefficients for intermediate steps (Butcher tableau).
     * @param b Weights for the final summation.
     * @param c Nodes (time fractions).
     */
    void setCoefficients(const Eigen::MatrixXd& a, const Eigen::VectorXd& b, const Eigen::VectorXd& c);

    /**
     * @brief Sets the coefficients for a user-defined Runge-Kutta method.
     * @param Rdr The reader object containing the coefficients.
     */
    void SetConfig(const Reader& Rdr) override;

    /**
     * @brief Performs a single step of the Runge-Kutta integration method.
     * @param y The current state vector.
     * @param t The current time.
     * @return The updated state vector after one Runge-Kutta step.
     */
    Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;  // Vectorized Step
};

#endif // RUNGEKUTTA_HH