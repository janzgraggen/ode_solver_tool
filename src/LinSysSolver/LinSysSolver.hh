//
// Created by janzgraggen on 27/11/2024.
//
#ifndef __LIN_SYS_SOLVER_HH__
#define __LIN_SYS_SOLVER_HH__

#include <Eigen/Dense>

/* -------------------------------------------------------------------------- */

/**
  * Abstract class for linear solvers.
  * This class is designed to be inherited by specific linear solver implementations.
  */

class LinSysSolver {
  /* ------------------------------------------------------------------------ */
  /* Members                                                                  */
  /* ------------------------------------------------------------------------ */
private:
  Eigen::MatrixXd A;  //!< The matrix A in the linear system A * x = b.
  Eigen::VectorXd b;  //!< The vector b in the linear system A * x = b.

  /* ------------------------------------------------------------------------ */
  /* Methods                                                                  */
  /* ------------------------------------------------------------------------ */

public:
  // constructor 
  LinSysSolver();

  // destructor
  virtual ~LinSysSolver();

  // setters 
  void SetA(const Eigen::MatrixXd& A);
  
  void SetB(const Eigen::VectorXd& b);

  // getters
  const Eigen::MatrixXd& GetA() const;

  const Eigen::VectorXd& GetB() const;

  // Solves the linear system A * x = b using the stored system data.
  virtual Eigen::VectorXd Solve() = 0;
};

/* -------------------------------------------------------------------------- */
#endif // __LIN_SYS_SOLVER_HH__