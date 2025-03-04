/**
 * @file AdamsBashforth.cc
 * @brief Implementation of the AdamsBashforth class methods.
 *
 * This source file contains the method definitions for the `AdamsBashforth` class,
 * which implements the Adams-Bashforth method for solving explicit time integration.
 * It supports variable orders from 1 to 4 and allows custom coefficients to be specified.
 *
 * Author: natha
 * Date: 27/11/2024
 */

#include "AdamsBashforth.hh"
#include <stdexcept>
#include <utility>

/**
 * @brief Constructor for AdamsBashforth with a logger reference.
 *
 * @param logger_ Reference to the Logger instance.
 */
AdamsBashforth::AdamsBashforth(Logger& logger_) : Explicit(logger_) {}

/**
 * @brief Constructor for AdamsBashforth with a specified maximum order.
 *
 * @param logger_ Reference to the Logger instance.
 * @param maxOrder The desired order of the Adams-Bashforth method (1 to 4).
 */
AdamsBashforth::AdamsBashforth(Logger& logger_, const int maxOrder)
    : Explicit(logger_), maxOrder(maxOrder), customCoefficients(Eigen::VectorXd()) {
    if (maxOrder < 1 || maxOrder > 4) {
        logger->error("{in AdamsBashforth::AdamsBashforth()} Supported orders are 1 through 4. Order given: " + std::to_string(maxOrder));
    }
    coefficients = generateCoefficients(1); // Start with order 1 coefficients
}

/**
 * @brief Constructor for AdamsBashforth with custom coefficients provided.
 *
 * @param logger_ Reference to the Logger instance.
 * @param customCoefficients A vector containing custom coefficients for the method.
 */
AdamsBashforth::AdamsBashforth(Logger& logger_, const Eigen::VectorXd& customCoefficients)
    : Explicit(logger_),
      maxOrder(customCoefficients.size()),
      coefficients(generateCoefficients(1)),
      customCoefficients(customCoefficients) {}

/**
 * @brief Generate coefficients for the Adams-Bashforth method based on the given order.
 *
 * @param order The desired order (1 to 4) for which coefficients are generated.
 * @return Eigen::VectorXd The generated coefficients for the specified order.
 */
Eigen::VectorXd AdamsBashforth::generateCoefficients(const int order) {
    switch (order) {
        case 1: return Eigen::VectorXd::Constant(1, 1.0);
        case 2: return (Eigen::VectorXd(2) << 3.0 / 2.0, -1.0 / 2.0).finished();
        case 3: return (Eigen::VectorXd(3) << 23.0 / 12.0, -16.0 / 12.0, 5.0 / 12.0).finished();
        case 4: return (Eigen::VectorXd(4) << 55.0 / 24.0, -59.0 / 24.0, 37.0 / 24.0, -9.0 / 24.0).finished();
        default:
            logger->error("{in AdamsBashforth::generateCoefficients()} Unsupported order for coefficients: " + std::to_string(order));
            return {};
    }
}

/**
 * @brief Set the maximum order for the Adams-Bashforth method.
 *
 * @param maxOrder The desired maximum order of the Adams-Bashforth method.
 */
void AdamsBashforth::setOrder(const int maxOrder) {
    if (maxOrder < 1 || maxOrder > 4) {
        logger->error("{in AdamsBashforth::setOrder()} Supported orders are 1 through 4. Order given: " + std::to_string(maxOrder));
    }
    this->maxOrder = maxOrder;
}

/**
 * @brief Set custom coefficients for the Adams-Bashforth method.
 *
 * @param customCoefficients A vector containing custom coefficients for the method.
 */
void AdamsBashforth::setCustomCoefficients(Eigen::VectorXd customCoefficients) {
    this->customCoefficients = std::move(customCoefficients);
}

/**
 * @brief Configure the Adams-Bashforth method based on settings provided by a `Reader` object.
 *
 * @param Rdr The `Reader` instance containing configuration settings.
 */
void AdamsBashforth::setConfig(const Reader& Rdr) {
    setGlobalConfig(Rdr);  // Call the base class method
    ExplicitSettings settings = Rdr.getExplicitSettings();
    if (settings.AdamsBashforth_max_order.has_value()) {
        setOrder(settings.AdamsBashforth_max_order.value());
    } else if (settings.AdamsBashforth_coefficients_vector.has_value()) {
        setCustomCoefficients(settings.AdamsBashforth_coefficients_vector.value());
    } else {
        logger->error("{in AdamsBashforth::setConfig()} Missing AdamsBashforth settings");
    }
}

/**
 * @brief Perform a single integration step using the Adams-Bashforth method.
 *
 * @param y The current state vector.
 * @param t The current time value.
 * @return Eigen::VectorXd The updated state vector after performing the integration step.
 */
Eigen::VectorXd AdamsBashforth::calcStep(const Eigen::VectorXd& y, double t) {
    const Eigen::VectorXd dydt = f_rhs(y, t);  // Compute the derivative at the current step

    // If history is empty, use order 1 as this is the first step
    if (history.empty()) {
        Eigen::VectorXd defaultCoefficients = generateCoefficients(1);
        Eigen::VectorXd result = y + getStepSize() * defaultCoefficients(0) * dydt;
        history.push_front(dydt);  // Update history with the current derivative
        return result;
    }

    // Determine the current order based on available history
    if (const int currentOrder = static_cast<int>(history.size()) + 1;
        customCoefficients.size() != 0 && currentOrder == customCoefficients.size()) {
        coefficients = customCoefficients;
    } else if (currentOrder <= 4) {
        coefficients = generateCoefficients(currentOrder);
    }

    // Perform the Adams-Bashforth step
    Eigen::VectorXd result = y + getStepSize() * coefficients(0) * dydt;
    for (int i = 1; i < coefficients.size(); ++i) {
        result += getStepSize() * coefficients(i) * history[i - 1];
    }

    // Update history with the current derivative
    history.push_front(dydt);
    if (history.size() > maxOrder - 1 || history.size() == customCoefficients.size()) {
        history.pop_back();
    }

    return result;
}