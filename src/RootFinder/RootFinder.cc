//
// Created by janzgraggen on 27/11/2024.
//
#include "RootFinder.hh"

//constructor and destructor implementation
// Constructor 1: Only the function pointer F is passed
RootFinder::RootFinder(Eigen::VectorXd (*F_in)(Eigen::VectorXd))
    : tolerance(1e-6), maxIterations(1000), iterationCount(0), converged(false), status("Not started") {
    this->F = F_in;  // Initialize the function pointer F
}


RootFinder::RootFinder(Eigen::VectorXd (*F_in)(Eigen::VectorXd), double tol, int maxIter)
    : RootFinder(F_in) {  // Delegate to the first constructor
    setTolerance(tol);    // Set the custom tolerance
    setMaxIterations(maxIter);  // Set the custom maximum iterations
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
