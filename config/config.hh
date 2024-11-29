
#ifndef CONFIG_HH
#define CONFIG_HH

#include <Eigen/Dense>

Eigen::VectorXd f(const Eigen::VectorXd& y, double t) {
    // –––––––––––––––––––––––––––––––––––––––––––––
    // your function here:
    Eigen::VectorXd f_out(y.size());
    for (int i = 0; i < y.size(); ++i) {
        f_out(i) = y(i);  
    }
    // –––––––––––––––––––––––––––––––––––––––––––––
    return f_out;
}

#endif // CONFIG_HH