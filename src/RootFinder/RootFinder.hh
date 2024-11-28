#ifndef __ROOT_FINDER__HH__
#define __ROOT_FINDER__HH__

#include <Eigen/Dense>
#include <string>
#include <iostream>

#include <cmath> 
#include <stdexcept>



class RootFinder {
/* ------------------------------------------------------------------------ */
/* Members                                                                  */
/* ------------------------------------------------------------------------ */
private:
    Eigen::VectorXd (*F)(Eigen::VectorXd y1);     // The root function

    double tolerance;             // Error tolerance
    int maxIterations;            // Maximum number of iterations
    int iterationCount;           // Number of iterations performed
    bool converged;               // Convergence flag
    std::string status;           // Status description
    Eigen::VectorXd initialGuess; // Initial guess for the root
    Eigen::VectorXd lastSolution; // Store the last solution

/* ------------------------------------------------------------------------ */
/* Methods                                                                  */
/* ------------------------------------------------------------------------ */
public:
    // Constructor
    RootFinder(Eigen::VectorXd (*F_in)(Eigen::VectorXd));
    RootFinder(Eigen::VectorXd (*F_in)(Eigen::VectorXd), double tol, int maxIter);
    // Destructor
    virtual ~RootFinder();

    // Setters
    void setTolerance(double tol);
    void setMaxIterations(int maxIter);
    void setInitialGuess(const Eigen::VectorXd& guess);
    void setIterationCount(int iter);

    // Getters
    int getIterationCount() const;
    double getTolerance() const;
    int getMaxIterations() const;
    Eigen::VectorXd getInitialGuess() const;
    Eigen::VectorXd getLastSolution() const;
    bool isConverged() const;
    std::string getStatus() const;

    // Function that calls the root function F (getter)
    Eigen::VectorXd callF(Eigen::VectorXd y1);

    // Virtual function to solve the root-finding problem (to be overridden)
    virtual Eigen::VectorXd Solve() = 0;

protected:
    // Log a step (can be overridden for custom logging)
    virtual void logStep(int iteration, const Eigen::VectorXd& currentSolution, double currentError);

};

#endif //__ROOT_FINDER__HH__
