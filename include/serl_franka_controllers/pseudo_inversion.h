#pragma once

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

namespace serl_franka_controllers {

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

inline void pseudoInverse(const Eigen::Matrix<double, 7, 6>& M_,
                          Eigen::Matrix<double, 6, 7>& M_pinv_,
                          bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::Matrix<double, 7, 6>> svd(M_,
                                                     Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 6, 1> sing_vals_ = svd.singularValues();
  Eigen::Matrix<double, 7, 6> S_ = Eigen::Matrix<double, 7, 6>::Zero();

  for (int i = 0; i < 6; i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = svd.matrixV() * S_.transpose() * svd.matrixU().transpose();
}

}  // namespace serl_franka_controllers