//
// Created by natha on 27/11/2024.
//

#include "AdamsBashforth.hh"
#include <stdexcept>
#include <iostream>


AdamsBashforth::AdamsBashforth() {}
AdamsBashforth::AdamsBashforth(int maxOrder) : maxOrder(maxOrder), customCoefficients(Eigen::VectorXd()) {
    if (maxOrder < 1 || maxOrder > 4) {
        throw std::invalid_argument("Supported orders are 1 through 4.");
    }
    coefficients = generateCoefficients(1); // Start with order 1 coefficients
}

AdamsBashforth::AdamsBashforth(const Eigen::VectorXd customCoefficients)
    : maxOrder(customCoefficients.size()),
      customCoefficients(customCoefficients),
      coefficients(generateCoefficients(1)) {}

Eigen::VectorXd AdamsBashforth::generateCoefficients(const int order) {
    switch (order) {
        case 1: return Eigen::VectorXd::Constant(1, 1.0);
        case 2: return (Eigen::VectorXd(2) << 3.0 / 2.0, -1.0 / 2.0).finished();
        case 3: return (Eigen::VectorXd(3) << 23.0 / 12.0, -16.0 / 12.0, 5.0 / 12.0).finished();
        case 4: return (Eigen::VectorXd(4) << 55.0 / 24.0, -59.0 / 24.0, 37.0 / 24.0, -9.0 / 24.0).finished();
        default:
            throw std::invalid_argument("Unsupported order for coefficients.");
    }
}

void AdamsBashforth::SetMaxOrder(int maxOrder) {
    if (maxOrder < 1 || maxOrder > 4) {
        throw std::invalid_argument("Supported orders are 1 through 4.");
    }
    this->maxOrder = maxOrder;
}

void AdamsBashforth::SetCustomCoefficients(Eigen::VectorXd customCoefficients) {
    this->customCoefficients = customCoefficients;
}


void AdamsBashforth::SetConfig(const Reader& Rdr) {
    SetGlobalConfig(Rdr); // Call the base class method

    if (Rdr.getExplicitSettings().AdamsBashforth_max_order.has_value()) {
        SetMaxOrder( Rdr.getExplicitSettings().AdamsBashforth_max_order.value() );

    }else if (Rdr.getExplicitSettings().AdamsBashforth_coefficients_vector.has_value()) {
        SetCustomCoefficients( Rdr.getExplicitSettings().AdamsBashforth_coefficients_vector.value());
    } else {
        throw std::invalid_argument("Invalid AdamsBashforth settings");
    }
}

Eigen::VectorXd AdamsBashforth::Step(const Eigen::VectorXd& y, double t) {
  const Eigen::VectorXd dydt = f_rhs(y, t); // Compute the derivative at the current step

    // If history is empty, this is the first step. Use order 1.
    if (history.empty()) {
        Eigen::VectorXd defaultCoefficients = generateCoefficients(1);
        Eigen::VectorXd result = y + GetStepSize() * defaultCoefficients(0) * dydt;
        history.push_front(dydt); // Update history with the current derivative
        return result;
    }

    // Determine the current order based on available history
    const int currentOrder = static_cast<int>(history.size()) + 1;

    // Update coefficients dynamically if not using a custom vector
    if (customCoefficients.size() != 0 && currentOrder == customCoefficients.size()) {
        coefficients = customCoefficients;
    } else if (currentOrder <= 4) {
        coefficients = generateCoefficients(currentOrder);
    }

    // Perform the Adams-Bashforth step
    Eigen::VectorXd result = y + GetStepSize() * coefficients(0) * dydt;
    for (int i = 1; i < coefficients.size(); ++i) {
        result += GetStepSize() * coefficients(i) * history[i - 1];
    }

    // Update history with the current derivative
    history.push_front(dydt);
    if (history.size() > maxOrder - 1 || history.size() == customCoefficients.size()) {
        history.pop_back();
    }

    return result;
}