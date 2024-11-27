//
// Created by natha on 27/11/2024.
//

#include "AdamsBashforth.hpp"
#include <stdexcept>

AdamsBashforth::AdamsBashforth(int maxOrder) : maxOrder(maxOrder) {
    if (maxOrder < 1 || maxOrder > 4) {
        throw std::invalid_argument("Supported orders are 1 through 4.");
    }
    coefficients = generateCoefficients(1); // Start with order 1 coefficients
}

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

Eigen::VectorXd AdamsBashforth::Step(const Eigen::VectorXd& y, double t) {
    const int currentOrder = std::min(maxOrder, static_cast<int>(history.size()) + 1);

    if (history.size() < currentOrder) {
        coefficients = generateCoefficients(currentOrder); // Adjust coefficients for the current order
    }

    Eigen::VectorXd result = y;
    for (int i = 0; i < currentOrder; ++i) {
        result += GetStepSize() * coefficients(i) * history[i];
    }

    return result;
}

void AdamsBashforth::updateHistory(const Eigen::VectorXd& y) {
    history.push_front(y);

    if (history.size() > static_cast<size_t>(maxOrder)) {
        history.pop_back();
    }
}