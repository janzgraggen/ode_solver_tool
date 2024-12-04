
#ifndef CONFIG_HH
#define CONFIG_HH

#include <Eigen/Dense>

Eigen::VectorXd __f__(const Eigen::VectorXd& y, double t) {
    // –––––––––––––––––––––––––––––––––––––––––––––
    // your function here:
    Eigen::VectorXd f_out(y.size());
    f_out(0) = 0.0;
    f_out(1) = y(1);
    f_out(2) = 2.0 * t;
    // –––––––––––––––––––––––––––––––––––––––––––––
    return f_out;
}

#endif // CONFIG_HH