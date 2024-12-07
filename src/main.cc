#include <iostream>
#include "Ode/AdamsBashforth.hh"
#include "Ode/ForwardEuler.hh"
#include "Ode/RungeKutta.hh"
#include "Reader/Reader.hh"
#include "Runner/Runner.hh"

int main() {
    Runner runner("../config/main/ODE_config.yaml");
    const Eigen::VectorXd finalSolution = runner.run();  
    std::cout << "Final solution:"  << std::endl;  
    for (int i = 0; i < finalSolution.size(); ++i) {
        std::cout << "  y" << i + 1 << ": " << finalSolution(i) << std::endl;
    }
    return 0;
}