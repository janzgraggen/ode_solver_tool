//
// Created by natha on 27/11/2024.
//

#ifndef ADAMSBASHFORTH_HH
#define ADAMSBASHFORTH_HH

#include "Explicit.hh"
#include <deque>

/**
 * @class AdamsBashforth
 * @brief A class to solve systems of ordinary differential equations (ODEs) using the Adams-Bashforth method with Eigen::VectorXd.
 */
class AdamsBashforth : public Explicit {
private:
 int maxOrder; ///< Maximum order of the Adams-Bashforth method.
 Eigen::VectorXd coefficients; ///< Coefficients for the current order of the method.
 Eigen::VectorXd customCoefficients;
 std::deque<Eigen::VectorXd> history; ///< History of previous derivatives.

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
 explicit AdamsBashforth(Logger& logger_);
 explicit AdamsBashforth(Logger& logger_,int maxOrder);
 explicit AdamsBashforth(Logger& logger_,const Eigen::VectorXd& customCoefficients);

 void SetMaxOrder(int maxOrder);
 void SetCustomCoefficients(Eigen::VectorXd customCoefficients);

 /**
  * @brief Sets the coefficients for a predefined Adams-Bashforth method.
  * @param maxOrder The maximum order of the Adams-Bashforth method (1, 2, 3, or 4).
  * @throws std::invalid_argument if the order is unsupported.
  */   
void SetConfig(const Reader& Rdr) override;

 /**
  * @brief Advances the solution using the Adams-Bashforth method.
  * @param y Current solution vector.
  * @param t Current time.
  * @return The solution at the next step.
  */
 Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;
};

#endif // ADAMSBASHFORTH_HH
