//
// Created by janzgraggen on 27/11/2024.
//
#include "RootFinder.hh"
using F_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&)> ;


//constructor and destructor implementation
// Constructor 1: Accepts an std::function instead of function pointer
RootFinder::RootFinder(F_TYPE F_in)
    : tolerance(1e-6), maxIterations(1000), iterationCount(0), converged(false), status("Not started"), F(F_in) {
}

// Constructor 2: Accepts an std::function and additional parameters for tolerance and maxIterations
RootFinder::RootFinder(F_TYPE F_in, double tol, int maxIter)
    : RootFinder(F_in) {  // Delegates to the first constructor
    setTolerance(tol);    // Set custom tolerance
    setMaxIterations(maxIter);  // Set custom maximum iterations
}



RootFinder::~RootFinder() {}

//setters
void RootFinder::setTolerance(double tol) {
    tolerance = tol;
}

void RootFinder::setMaxIterations(int maxIter) {
    maxIterations = maxIter;
}

void RootFinder::setInitialGuess(const Eigen::VectorXd& guess) {
    initialGuess = guess;
}


void RootFinder::setIterationCount(int iter) {
    iterationCount = iter;
}

//getters
double RootFinder::getTolerance() const {
    return tolerance;
}


int RootFinder::getIterationCount() const {
    return iterationCount;
}

int RootFinder::getMaxIterations() const {
    return maxIterations;
}

Eigen::VectorXd RootFinder::getLastSolution() const {
    return lastSolution;
}

Eigen::VectorXd RootFinder::getInitialGuess() const {
    return initialGuess;
}

bool RootFinder::isConverged() const {
    return converged;
}

std::string RootFinder::getStatus() const {
    return status;
}

// The function F is defined in the base class 
Eigen::VectorXd RootFinder::callF(Eigen::VectorXd y1) {
    return F(y1);
}

void RootFinder::logStep(int iteration, const Eigen::VectorXd& currentSolution, double currentError) {
    // Default log output (this can be overridden in derived classes)
    std::cout << "Iteration " << iteration << ": Error = " << currentError << std::endl;
}
