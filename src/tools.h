#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;


class Tools {
public:
  // Constructor
  Tools();

  // Destructor.
  virtual ~Tools();

  // A helper method to calculate RMSE.
  VectorXd CalculateRMSE(const std::vector<VectorXd>& estimations,
                         const std::vector<VectorXd>& ground_truth);
};

#endif // TOOLS_H_
