#include "Runner.hh"
#include <iostream>
#include <../Ode/ForwardEuler.hh>
#include <../Ode/RungeKutta.hh>
#include <../Ode/AdamsBashforth.hh>
#include <../Ode/BackwardEuler.hh>

#include "../LinSysStruct.hh"

//constructor
Runner::Runner(const std::string& config_file) : 
    config_file(config_file), Rdr(config_file) {}


//destructor
Runner::~Runner() {}

//run method

void Runner::run() {

if (Rdr.getSolverType() == "Explicit") {
    if (Rdr.getExplicitSettings().method == "ForwardEuler"){

    } else if (Rdr.getExplicitSettings().method == "RungeKutta") {
        if (Rdr.getExplicitSettings().RungeKutta_order.has_value()) {

        } else if (
            Rdr.getExplicitSettings().RungeKutta_order.has_value()
            &&  Rdr.getExplicitSettings().RungeKutta_coefficients_a.has_value()
            &&  Rdr.getExplicitSettings().RungeKutta_coefficients_b.has_value()) {

        } else {
            std::cout << "Invalid RungeKutta settings" << std::endl;
        }
        
    
    } else if (Rdr.getExplicitSettings().method == "AdamsBashforth") {
        if (Rdr.getExplicitSettings().AdamsBashforth_max_order.has_value()){

        } else if (Rdr.getExplicitSettings().AdamsBashforth_coefficients_vector.has_value()) {

        } else {
            std::cout << "Invalid AdamsBashforth settings" << std::endl;
        }

    } else {
        std::cout << "Invalid explicit solver method" << std::endl;
    }


} else if (Rdr.getSolverType() == "Implicit") {
    if (Rdr.getImplicitSettings().method == "BackwardEuler") {
        if (Rdr.getImplicitSettings().rhs_is_linear) {
            
            BackwardEuler Solver;
            Solver.SetRhsIsLinear(Rdr.getImplicitSettings().rhs_is_linear);
            Solver.SetRhsSystem(Rdr.getImplicitSettings().rhs_system.value()); // value() to get the value of the optional 
            Solver.SolveODE(std::cout);

        } else {


        }



    } else {
        std::cout << "Invalid implicit solver method" << std::endl;
    }


}else {
    std::cout << "Invalid solver type" << std::endl;


}



}