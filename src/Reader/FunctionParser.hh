#ifndef __FUNCTIONPARSER_HH__
#define __FUNCTIONPARSER_HH__

#include <vector>
#include <string>
#include <functional>
#include <Eigen/Dense>
#include "muParser.h"

using strList = std::vector<std::string>;
using f_TYPE = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;

class FunctionParser {
private:
    strList f_i;             // Function strings
    std::vector<mu::Parser> parsers;          // muParser instances for each function
    std::vector<double> variables;            // Shared variables for the parsers (t, y1, ..., yn)
    size_t n;                                 // Dimension of the input/output vector

public:
    // Constructor
    FunctionParser(const strList& functions);

    // Destructor
    ~FunctionParser() = default;

    // Evaluates the function at a given input
    Eigen::VectorXd evaluate(const Eigen::VectorXd& input);

    // Returns the callable function that takes t and y as input
    f_TYPE getFunction() const;

    // Returns the number of functions (dimension of output)
    size_t dimension() const { return n; }
};

#endif // FUNCTIONPARSER_HH
