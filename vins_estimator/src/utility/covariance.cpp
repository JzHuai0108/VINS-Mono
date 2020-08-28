#include "covariance.h"

#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include <ros/ros.h>
#include "tic_toc.h"

CovarianceSolver::CovarianceSolver() : kPoseBlockIndex(WINDOW_SIZE) {
  
}

CovarianceSolver::CovarianceSolver(const std::string outputFile): kPoseBlockIndex(WINDOW_SIZE) {
  setOutput(outputFile);
}

void CovarianceSolver::setOutput(const std::string outputFile) {
  stream_.open(outputFile, std::ofstream::out);
  if (!stream_.is_open()) {
    ROS_INFO("Error opening covariance output file %s!", outputFile.c_str());
  }
}

CovarianceSolver::~CovarianceSolver() {
  stream_.close();
}

bool CovarianceSolver::compute(ceres::Problem& problem, 
    double para_Pose[][SIZE_POSE],
    Eigen::Matrix<double, SIZE_POSE-1, 1>* std_pose) const {
  TicToc t_cov;
  Eigen::Matrix<double, SIZE_POSE-1, SIZE_POSE-1, Eigen::RowMajor> cov_pose;
  ceres::Covariance::Options cov_options;
  cov_options.algorithm_type = ::ceres::SPARSE_QR;
  cov_options.num_threads = 1;
  cov_options.min_reciprocal_condition_number = 1e-12;
  cov_options.apply_loss_function = true;
  ceres::Covariance covariance(cov_options);
  
  std::vector<std::pair<const double*, const double*>> covariance_blocks;
  covariance_blocks.push_back(std::make_pair(para_Pose[kPoseBlockIndex], para_Pose[kPoseBlockIndex]));
  bool status = covariance.Compute(covariance_blocks, &problem);
  if (status) {
      covariance.GetCovarianceBlockInTangentSpace(
        para_Pose[kPoseBlockIndex], para_Pose[kPoseBlockIndex], cov_pose.data());
      // ROS_INFO_STREAM("cov: " << cov_pose);
      *std_pose = cov_pose.diagonal().cwiseSqrt();
  } else {
      ROS_WARN("Covariance Compute failed!");
      std_pose->setZero();
  }
  t_cov.toc();
  return status;
}

void CovarianceSolver::dump(const Eigen::Matrix<double, SIZE_POSE-1, 1>& std_pose, double stamp) {
  stream_ << std::setprecision(18) << stamp << std::setprecision(6) 
      << std_pose.transpose() << std::endl;
}
