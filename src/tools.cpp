#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;


Tools::Tools() {}
Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const std::vector<VectorXd>& estimations,
                              const std::vector<VectorXd>& ground_truth) {
  // Calculate the RMSE.
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // Check the validity of the following inputs:
  //  - the estimation vector size should not be zero
  //  - the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    std::cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // Accumulate squared residuals
  for (size_t i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    // Coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();

  // Calculate the squared root
  rmse = rmse.array().sqrt();

  // Return the result
  return rmse;
}
