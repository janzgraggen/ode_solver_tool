/**
 * @file AdamsBashforth.hh
 * @brief Header file for the AdamsBashforth class.
 *
 * This header defines the `AdamsBashforth` class, which implements the Adams-Bashforth method
 * for solving systems of ordinary differential equations (ODEs) using the Eigen library.
 * It supports multiple orders (1 through 4) and allows custom coefficients to be specified.
 * The method is built on top of the `Explicit` class.
 *
 * Author: natha
 * Date: 27/11/2024
 */

#ifndef ADAMSBASHFORTH_HH
#define ADAMSBASHFORTH_HH

#include "Explicit.hh"
#include <deque>

/**
 * @class AdamsBashforth
 * @brief A class to solve systems of ordinary differential equations (ODEs) using the Adams-Bashforth method with Eigen::VectorXd.
 *
 * This class extends the `Explicit` base class and provides the implementation
 * for solving ODEs using the Adams-Bashforth time integration method. It stores
 * a history of previous derivatives and allows specifying custom coefficients.
 */
class AdamsBashforth : public Explicit {
private:
    int maxOrder;  ///< Maximum order of the Adams-Bashforth method (1 to 4).
    Eigen::VectorXd coefficients;  ///< Coefficients for the current order of the method.
    Eigen::VectorXd customCoefficients;  ///< Custom coefficients provided by the user.
    std::deque<Eigen::VectorXd> history;  ///< Stores the history of previous derivatives.

    /**
     * @brief Generates coefficients for a specific order of the Adams-Bashforth method.
     *
     * Generates predefined coefficients for supported orders (1 through 4).
     * @param order The desired order (1 to 4).
     * @return An Eigen::VectorXd containing the coefficients for the specified order.
     * @throws std::invalid_argument if the requested order is unsupported.
     */
    Eigen::VectorXd generateCoefficients(int order);

public:
    /**
     * @brief Constructor for the Adams-Bashforth method with a logger reference.
     * @param logger_ Reference to the Logger instance.
     */
    explicit AdamsBashforth(Logger& logger_);

    /**
     * @brief Constructor with a specified maximum order for the Adams-Bashforth method.
     * @param logger_ Reference to the Logger instance.
     * @param maxOrder The desired maximum order of the Adams-Bashforth method (1 to 4).
     */
    explicit AdamsBashforth(Logger& logger_, int maxOrder);

    /**
     * @brief Constructor with custom coefficients provided by the user.
     * @param logger_ Reference to the Logger instance.
     * @param customCoefficients A vector containing custom coefficients for the method.
     */
    explicit AdamsBashforth(Logger& logger_, const Eigen::VectorXd& customCoefficients);

    /**
     * @brief Sets the maximum order for the Adams-Bashforth method.
     * @param maxOrder The desired maximum order (1 to 4).
     * @throws std::invalid_argument if the provided order is unsupported.
     */
    void SetMaxOrder(int maxOrder);

    /**
     * @brief Sets custom coefficients for the Adams-Bashforth method.
     * @param customCoefficients A vector containing custom coefficients.
     */
    void SetCustomCoefficients(Eigen::VectorXd customCoefficients);

    /**
     * @brief Configures the method based on provided settings from a `Reader` object.
     *
     * Reads and sets the method's configuration, including `maxOrder` or custom coefficients,
     * based on the `Reader`'s provided settings.
     * @param Rdr The `Reader` instance containing configuration details.
     */
    void SetConfig(const Reader& Rdr) override;

    /**
     * @brief Advances the solution using the Adams-Bashforth method.
     *
     * Performs a single integration step using the Adams-Bashforth time integration method.
     * @param y The current state vector representing the system's state.
     * @param t The current time value.
     * @return The updated state vector after the integration step.
     */
    Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;
};

#endif // ADAMSBASHFORTH_HH