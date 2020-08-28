#pragma once

#include <string>

#include <ceres/ceres.h>

#include "../parameters.h"

class CovarianceSolver {
 public:
  CovarianceSolver();

  explicit CovarianceSolver(const std::string outputFile);

  CovarianceSolver(const CovarianceSolver& solver) = delete;
  
  virtual ~CovarianceSolver();
  
  void setOutput(const std::string outputFile);

  bool compute(ceres::Problem& problem, double para_Pose[][SIZE_POSE], Eigen::Matrix<double, SIZE_POSE-1, 1>* std_pose) const;
  
  void dump(const Eigen::Matrix<double, SIZE_POSE-1, 1>& std_pose, double stamp);

  const int kPoseBlockIndex;

 private:
  std::ofstream stream_;

};
