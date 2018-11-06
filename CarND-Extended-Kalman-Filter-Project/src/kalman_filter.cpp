#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

MatrixXd CalculateJacobian(const VectorXd &x_state) {
  MatrixXd Hj(3, 4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);
  //check division by zero
  if(fabs(c1) < 0.00001) {
    printf("CalculateJacobian () - Error - Division by Zero\n");
    return Hj;
  }
  //compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
  -(py / c1), (px / c1), 0, 0,
  py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;
  return Hj;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  auto Hj = CalculateJacobian(x_);
  VectorXd z_pred = cart2pol(x_);
  VectorXd y = z - z_pred;
  if(y(1) < -PI) {
    y(1) += PI * 2;
  }else if(y(1) > PI) {
    y(1) -= PI * 2;
  }

  MatrixXd Ht = Hj.transpose();
  MatrixXd S = Hj * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * Hj) * P_;
}

Eigen::VectorXd KalmanFilter::cart2pol(const Eigen::VectorXd &z) {
  auto ret = VectorXd(3);
  float px = z(0);
  float py = z(1);
  float vx = z(2);
  float vy = z(3);
  //pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  ret << c2, atan2(py, px), (px * vx + py * vy) / c2;
  return ret;
}
