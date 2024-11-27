//
// Created by natha on 27/11/2024.
//

#ifndef ADAMSBASHFORTH_HPP
#define ADAMSBASHFORTH_HPP

#include "Explicit.hpp"
#include <deque>

/**
 * @class AdamsBashforth
 * @brief A class to solve systems of ordinary differential equations (ODEs) using the Adams-Bashforth method with Eigen::VectorXd.
 */
class AdamsBashforth final : public Explicit {
private:
    int maxOrder; ///< Maximum order of the Adams-Bashforth method.
    Eigen::VectorXd coefficients; ///< Coefficients for the current order of the method.
    std::deque<Eigen::VectorXd> history; ///< History of previous steps.

    /**
     * @brief Generates coefficients for a specific order of the Adams-Bashforth method.
     * @param order The order for which coefficients are needed.
     * @return The coefficients as an Eigen::VectorXd.
     * @throws std::invalid_argument if the order is unsupported.
     */
    [[nodiscard]] static Eigen::VectorXd generateCoefficients(int order);

public:
    /**
     * @brief Constructor for Adams-Bashforth methods by order.
     * @param maxOrder The maximum order of the Adams-Bashforth method (1, 2, 3, or 4).
     */
    explicit AdamsBashforth(int maxOrder);

    /**
     * @brief Advances the solution using the Adams-Bashforth method.
     * @param y Current solution vector.
     * @param t Current time.
     * @return The solution at the next step.
     * @throws std::runtime_error if insufficient history exists for the current step.
     */
    Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;

    /**
     * @brief Updates the history of past derivatives.
     * @param y The derivative at the current step.
     */
    void updateHistory(const Eigen::VectorXd& y);
};

#endif // ADAMSBASHFORTH_HPP
