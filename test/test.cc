#include <gtest/gtest.h>
#include <../../eigen/Eigen/Dense>
#include <cmath>
#include "../src/Reader/Reader.hh"
#include "../src/Runner/Runner.hh"
#include "../src/Logger/Logger.hh"

//// TESTING THE PARSING

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


TEST(TestReaderAndRunnerWithImplBwdEulerLinear, BasicTest) {
  auto* loggerPtr = new Logger(0);
   // Create the Runner object using the test config file
   Runner runner(*loggerPtr, "../config/test/config_test_ImplBwdEulerLinear.yaml");

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