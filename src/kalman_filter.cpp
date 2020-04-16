#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
  /**
   * TODO: predict the state
   */
  // cout << "Predicting state" << endl;

  // Calculate x'
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  // Calculate P'
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

  // Update state with standard Kalman Filter
  // cout << "Updating state with standard Kalman Filter" << endl;

  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // New estimate of the state and covariance
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

  // Update state with Extended Kalman Filter
  // cout << "Updating state with Extended Kalman Filter" << endl;

  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  // Calculate rho, phi and rho_dot
  float rho = sqrt(px * px + py * py);
  float phi = atan2(py, px);

  // Check divison by zero
  if( rho == 0. ){
    return;
  }
  float rho_dot = (px * vx + py * vy) / rho;

  // Set hx from rho, phi, rho_dot
  VectorXd h = VectorXd(3);
  h << rho, phi, rho_dot;

  VectorXd y = z - h;

  // Make sure the angle of y is always between -pi to pi
  while(y[1] > M_PI || y[1] < -M_PI){
    if(y[1] > M_PI){
      y[1] -= 2 * M_PI;
    } else if (y[1] < M_PI){
      y[1] += 2 * M_PI;
    }
  }

  // Calculate Ht, S, Si, PHt and K
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // Make the new estimate of x' and P'
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
