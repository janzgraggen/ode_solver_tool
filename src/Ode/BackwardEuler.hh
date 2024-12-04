#include "Implicit.hh"

using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;

class BackwardEuler : public Implicit {


public: 
 // the function F (y1, others)
    virtual Eigen::VectorXd F(Eigen::VectorXd y1, Eigen::VectorXd y0,double t0);
    // a function that returns a passable F function
    virtual F_TYPE makeFstep(Eigen::VectorXd y0,double t0) override;

    void SetConfig(const Reader& Rdr) override;

    Eigen::VectorXd LinStep(const Eigen::VectorXd y, double t) override;
    
};