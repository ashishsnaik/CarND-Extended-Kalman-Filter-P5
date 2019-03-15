#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
   * predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * update the state by using Kalman Filter equations
   * this function is called for LIDAR measurements
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new state prediction/estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * update the state by using Extended Kalman Filter equations
   * this function is called for RADAR measurements
   */

  // get the current predicted state (x')
  float px = x_(0); // x position
  float py = x_(1); // y position
  float vx = x_(2); // x component of velocity
  float vy = x_(3); // y component of velocity

  // use the h(x') here to map the predicted state from cartesian to polar
  // coordinates, so we can calculate the error w.r.t actual measurement z.
  float rho = sqrt(px*px + py*py);
  // Kalman filter expects small angle values between the range -pi and pi.
  // atan2() functions returns values between -pi and pi.
  float phi = atan2(py, px);
  float rho_dot = (px*vx + py*vy)/rho;

  // create z_pred
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  // calculate error and update the state
  VectorXd y = z - z_pred;

  // ensure that the angle 'phi' in the 'z' vector is between -pi and pi
  // (M_PI below is a pi-constant in the cmath library)
  while (y(1) < -M_PI || y(1) > M_PI) {
    if (y(1) < -M_PI) {
      y(1) += 2 * M_PI;
    } else {
      y(1) -= 2 * M_PI;
    }
  }

  // H_ used is the H_jacobian as set by the
  // FusionEKF::ProcessMeasurement () function
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  // new state prediction/update
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
