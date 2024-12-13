#include <gtest/gtest.h>
#include <../../eigen/Eigen/Dense>
#include <cmath>
#include "../src/Reader/Reader.hh"
#include "../src/Runner/Runner.hh"
#include "../src/Logger/Logger.hh"
#include "../src/LinSysSolver/GaussElimSolve.hh"
#include "../src/LinSysSolver/LUSolve.hh"
#include "../src/RootFinder/NewtonRaphson.hh"

TEST(TestGaussElimSolve, BasicTest) {
    // Create a Logger instance
    auto* loggerPtr = new Logger(0);

    // Instantiate GaussElimSolve with the logger
    GaussElimSolve solver(*loggerPtr);

    // Define a test linear system Ax = b
    Eigen::MatrixXd A(3, 3);
    A << 3, 1, -1,
         2, 4, 1,
         -1, 2, 5;

    Eigen::VectorXd b(3);
    b << 4, 1, 1;

    // Set A and b in the solver (using inherited methods from LinSysSolver)
    solver.setA(A);
    solver.setB(b);

    // Solve the system
    Eigen::VectorXd x = solver.solveSys();

    // Compute the expected solution using Eigen for verification
    Eigen::VectorXd expectedSolution = A.colPivHouseholderQr().solve(b);

    // Compare the computed solution to the expected solution
    for (int i = 0; i < x.size(); ++i) {
        EXPECT_NEAR(x[i], expectedSolution[i], 1e-5);
    }

    // Clean up
    delete loggerPtr;
}

TEST(TestLUSolver, CompareWithGaussElim) {
    // Create a Logger instance
    auto* loggerPtr = new Logger(0);

    // Instantiate GaussElimSolve and LUSolve with the logger
    GaussElimSolve gaussSolver(*loggerPtr);
    LUSolve luSolver(*loggerPtr);

    // Define a test linear system Ax = b
    Eigen::MatrixXd A(3, 3);
    A << 3, 1, -1,
         2, 4, 1,
         -1, 2, 5;

    Eigen::VectorXd b(3);
    b << 4, 1, 1;

    // Set A and b in both solvers
    gaussSolver.setA(A);
    gaussSolver.setB(b);

    luSolver.setA(A);
    luSolver.setB(b);

    // Solve the system with both solvers
    Eigen::VectorXd xGauss = gaussSolver.solveSys();
    Eigen::VectorXd xLU = luSolver.solveSys();

    // Compare the solutions
    ASSERT_EQ(xGauss.size(), xLU.size()) << "Solution sizes differ!";
    for (int i = 0; i < xGauss.size(); ++i) {
        EXPECT_NEAR(xGauss[i], xLU[i], 1e-5);
    }

    // Clean up
    delete loggerPtr;
}


TEST(TestNewtonRaphson, BasicRootSolve) {
    // Create a Logger instance
    auto* loggerPtr = new Logger(0);

    // Define the function F(x) for which we want to find the root
    auto F = [](const Eigen::VectorXd& x) {
        Eigen::VectorXd F(1);
        F[0] = x[0] * x[0] - 2.0; // Root at sqrt(2)
        return F;
    };

    // Instantiate NewtonRaphson with the logger and the function F
    NewtonRaphson solver(*loggerPtr, F);

    // Configure the solver
    solver.setInitialGuess(Eigen::VectorXd::Constant(1, 1.0)); // Initial guess x = [1.0]
    solver.setTolerance(1e-5);
    solver.setMaxIterations(50);
    solver.setLinearSystemSolver("GaussianElimination");

    // Solve for the root
    Eigen::VectorXd root = solver.solveRoot();

    // Expected root is sqrt(2)
    double expectedRoot = std::sqrt(2.0);

    // Compare the computed root to the expected root
    EXPECT_NEAR(root[0], expectedRoot, 1e-5);

    // Clean up
    delete loggerPtr;
}


TEST(TestParsing, BasicTest){
  auto* loggerPtr = new Logger(0);
  Reader reader(*loggerPtr, "../config/test/config_test_Parser.yaml");
  f_TYPE f = reader.getFunction();
  Eigen::VectorXd vec = reader.getOdeSettings().initial_value;  // Vector of size 3
  double time = reader.getOdeSettings().initial_time;
  Eigen::VectorXd parsresult =  f(vec, time);
  Eigen::VectorXd expected(3);
  expected << 5, 2, 6;

  EXPECT_EQ(expected, parsresult);
  delete loggerPtr;

}

TEST(TestReaderAndRunnerWithExplFwdEuler, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_ExplFwdEuler.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*0.01); // expected solution of dy/dt = y
  EXPECT_NEAR(finalSolution[2], 1.0, 1.0*0.01); // expected solution of dy/dt = 2t
  delete loggerPtr;
}

TEST(TestReaderAndRunnerWithExplRK4, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_ExplRK4.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*pow(0.01, 4)); // expected solution of dy/dt = y
  EXPECT_NEAR(finalSolution[2], 1.0, 1.0*0.01); // expected solution of dy/dt = 2t
  delete loggerPtr;
}

TEST(TestReaderAndRunnerWithExplCustomRK, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_ExplCustomRK.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*pow(0.01, 4)); // expected solution of dy/dt = y
  EXPECT_NEAR(finalSolution[2], 1.0, 1.0*0.01); // expected solution of dy/dt = 2t
  delete loggerPtr;
}

TEST(TestReaderAndRunnerWithExplAB4, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_ExplAB4.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*0.0001); // expected solution of dy/dt = y
  EXPECT_NEAR(finalSolution[2], 1.0, 1.0*0.01); // expected solution of dy/dt = 2t
  delete loggerPtr;
}

TEST(TestReaderAndRunnerWithExplABcustomCoef, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_ExplABcustomCoef.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*0.01); // expected solution of dy/dt = y
  EXPECT_NEAR(finalSolution[2], 1.0, 1.0*0.01); // expected solution of dy/dt = 2t
  delete loggerPtr;
}


TEST(TestReaderAndRunnerWithImplBwdEulerLinearGauss, BasicTest) {
  auto* loggerPtr = new Logger(0);
   // Create the Runner object using the test config file
   Runner runner(*loggerPtr, "../config/test/config_test_ImplBwdEulerLinearGauss.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*0.01); // expected solution of dy/dt = y
  delete loggerPtr;
}

TEST(TestReaderAndRunnerWithImplBwdEulerLinearLU, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_ImplBwdEulerLinearLU.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*0.01); // expected solution of dy/dt = y
  delete loggerPtr;
}

TEST(TestReaderAndRunnerWithImplBwdEulerNonlinear, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_ImplBwdEulerNonlinear.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 1.0, 0.00001); // Expected to remain constant
  EXPECT_NEAR(finalSolution[1], std::exp(1), std::exp(1)*0.01); // expected solution of dy/dt = y
  EXPECT_NEAR(finalSolution[2], 1.0, 1.0*0.0101); // expected solution of dy/dt = 2t
  delete loggerPtr;
}

TEST(TestReaderAndRunnerWithPlanetRK4, BasicTest) {
  auto* loggerPtr = new Logger(0);
  // Create the Runner object using the test config file
  Runner runner(*loggerPtr, "../config/test/config_test_PlanetRK4.yaml");

  const Eigen::VectorXd finalSolution = runner.run();

  EXPECT_NEAR(finalSolution[0], 0.4, 0.01); // Expected to go back to initial
  EXPECT_NEAR(finalSolution[2], 0.0, 0.01); // Because 2-pi periodic
  delete loggerPtr;
}