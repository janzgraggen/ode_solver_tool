#include "Runner.hh"
#include <iostream>
#include <memory>
#include "../Ode/ForwardEuler.hh"
#include "../Ode/RungeKutta.hh"
#include "../Ode/AdamsBashforth.hh"
#include "../Ode/BackwardEuler.hh"

#include "../LinSysStruct.hh"

#include "../config/Config.hh"

//constructor
Runner::Runner(const std::string& config_file) : 
    config_file(config_file), Rdr(config_file) {}


//destructor
Runner::~Runner() {}

//run method

Eigen::VectorXd Runner::run() {
    if (Rdr.getSolverType() == "Explicit") {
        if (Rdr.getExplicitSettings().method == "ForwardEuler") {
            FwdEuler Solver;
            Solver.SetStepSize(Rdr.getOdeSettings().step_size);
            Solver.SetTimeInterval(Rdr.getOdeSettings().initial_time, Rdr.getOdeSettings().final_time);
            Solver.SetInitialValue(Rdr.getOdeSettings().initial_value);
            Solver.SetRightHandSide(f);
            return Solver.SolveODE(std::cout);

        } else if (Rdr.getExplicitSettings().method == "RungeKutta") {
            // If order is provided
            if (Rdr.getExplicitSettings().RungeKutta_order.has_value()) {
                RungeKutta Solver(Rdr.getExplicitSettings().RungeKutta_order.value());
                Solver.SetStepSize(Rdr.getOdeSettings().step_size);
                Solver.SetTimeInterval(Rdr.getOdeSettings().initial_time, Rdr.getOdeSettings().final_time);
                Solver.SetInitialValue(Rdr.getOdeSettings().initial_value);
                Solver.SetRightHandSide(f);
                return Solver.SolveODE(std::cout);

                // If coefficients a, b, c are provided
            } else if (Rdr.getExplicitSettings().RungeKutta_coefficients_a.has_value() &&
                       Rdr.getExplicitSettings().RungeKutta_coefficients_b.has_value() &&
                       Rdr.getExplicitSettings().RungeKutta_coefficients_c.has_value()) {
                RungeKutta Solver(Rdr.getExplicitSettings().RungeKutta_coefficients_a.value(),
                                  Rdr.getExplicitSettings().RungeKutta_coefficients_b.value(),
                                  Rdr.getExplicitSettings().RungeKutta_coefficients_c.value());
                Solver.SetStepSize(Rdr.getOdeSettings().step_size);
                Solver.SetTimeInterval(Rdr.getOdeSettings().initial_time, Rdr.getOdeSettings().final_time);
                Solver.SetInitialValue(Rdr.getOdeSettings().initial_value);
                Solver.SetRightHandSide(f);
                return Solver.SolveODE(std::cout);
                       } else {
                           std::cout << "Invalid RungeKutta settings" << std::endl;
                           return Eigen::VectorXd();  // Return empty vector for failure
                       }
        } else if (Rdr.getExplicitSettings().method == "AdamsBashforth") {
            if (Rdr.getExplicitSettings().AdamsBashforth_max_order.has_value()){
                AdamsBashforth Solver(Rdr.getExplicitSettings().AdamsBashforth_max_order.value());
                Solver.SetStepSize(Rdr.getOdeSettings().step_size);
                Solver.SetTimeInterval(Rdr.getOdeSettings().initial_time, Rdr.getOdeSettings().final_time);
                Solver.SetInitialValue(Rdr.getOdeSettings().initial_value);
                Solver.SetRightHandSide(f);
                return Solver.SolveODE(std::cout);
            } else {
                std::cout << "Invalid AdamsBashforth settings" << std::endl;
                return Eigen::VectorXd();  // Return empty vector for failure
            }
        } else {
            std::cout << "Invalid explicit solver method" << std::endl;
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else if (Rdr.getSolverType() == "Implicit") {
        if (Rdr.getImplicitSettings().method == "BackwardEuler") {
            BackwardEuler Solver;
            Solver.SetStepSize(Rdr.getOdeSettings().step_size);
            Solver.SetTimeInterval(Rdr.getOdeSettings().initial_time, Rdr.getOdeSettings().final_time);
            Solver.SetInitialValue(Rdr.getOdeSettings().initial_value);
            Solver.SetRightHandSide(f);
            Solver.SetRhsIsLinear(Rdr.getImplicitSettings().rhs_is_linear);
            if (Solver.GetRhsIsLinear()) {
                Solver.SetRhsSystem(Rdr.getImplicitSettings().rhs_system.value());
            } else {
                // Replace with nonlinear settings

            }
        } else {
            std::cout << "Invalid implicit solver method" << std::endl;
            return Eigen::VectorXd();  // Return empty vector for failure
        }

    } else {
        std::cout << "Invalid solver type" << std::endl;
        return Eigen::VectorXd();  // Return empty vector for failure
    }
}
