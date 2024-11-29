//
// Created by janzgraggen on 28/11/2024.
//

#include "OdeSolver.hpp"
#include "LinSysStruct.hh"

using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;

class Implicit : public OdeSolver {

private:
    bool rhs_is_linear; // flag to check if the right-hand side function is linear
    LinearSystem rhs_system; // linear system to solve for implicit methods
public:
    // destructor
    ~Implicit() override = default;

    // getter/ setter
    bool GetRhsIsLinear() const;
    void SetRhsIsLinear(bool);

    LinearSystem GetRhsSystem() const;
    void SetRhsSystem(LinearSystem);

    // override SetRightHandSide function to check if the right-hand side function is linear
    void SetRightHandSide(const f_TYPE& f) override;

   

    virtual Eigen::VectorXd LinStep(const Eigen::VectorXd y, double t) = 0; // changes in child classes
    Eigen::VectorXd NonLinStep(const Eigen::VectorXd y, double t);  //uses makeFstep that now also is virtual here

    virtual F_TYPE makeFstep(Eigen::VectorXd y0,double t0) = 0;

    // override Step function
    virtual Eigen::VectorXd Step(const Eigen::VectorXd& y, double t) override;

    // override Solve function
    void SolveODE(std::ostream& stream) override;
        
};
