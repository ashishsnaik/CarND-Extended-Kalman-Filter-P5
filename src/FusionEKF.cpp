#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {

  is_initialized_ = false;
  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // laser measurement fnc to map lidar state to measurement
  // Hj_ will be determined later based on the Jacobian
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //
  // initialize KalmanFilter by calling KalmanFilter::Init() with appropriate initializations
  //

  // state vector: initialize it with some values; the first 2 (px, py) will be modified
  // below, but the later 2 (vx, vy) we have no idea of what they are initially, so we
  // might get a high RMSE at the beginning, and which could accumulate over time, so we
  // will have to tune the last 2 values later to lower the RMSE.
  VectorXd x(4);
  x << 1, 1, 1, 1;

  // state transition matrix: for both, laser and radar, no need to
  // use the Jacobian F here as we are using linear measurement model
  MatrixXd F(4, 4);
  F << 1, 0, 1, 0,
       0, 1, 0, 1,
       0, 0, 1, 0,
       0, 0, 0, 1;

  // state uncertainty/covariance matrix: for both, laser and radar
  MatrixXd P(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;

  // process uncertainty/covariance matrix: for both, laser and radar
  // initialize it to zeros for now, we'll update it later based on time-delta
  MatrixXd Q(4, 4);
  Q << 0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0;

  // call KF Init () using the laser H and R for now
  // we'll update them in the ProcessMeasurement() below, as necessary
  ekf_.Init(x, P, F, H_laser_, R_laser_, Q);

  // initialize acceleration noise components to 9 as mentioned
  // in the guideline in ProcessMeasurement() function below
  noise_ax = 9.0;
  noise_ay = 9.0;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */

//  // FOR TESTING ONLY!
//  // USE THE BELOW COMMENTED CODE TO EVALUATE INDIVIDUAL 'LASER' AND 'RADAR'
//  // PERFORMANCES (IGNORE THE OTHER SENSOR'S MEASUREMENTS IN THE 'if' BELOW)
//  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
//    return;
//  }

  if (!is_initialized_) {

    // this is the first measurement our fusion module has received, so
    // just initialize the state ekf_.x_ with the first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // if it's a radar measurement, convert it from polar to
      // cartesian coordinates and initialize state
      // we only initialize the x-position and y-position (px and py) and
      // ignore the x and y components of velocity (vx and vy)
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      ekf_.x_(0) = rho * cos(phi);
      ekf_.x_(1) = rho * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // initialize state
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // initialize the previous timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  // compute the time elapsed (dt) between the current and
  // previous measurements, expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // update F matrix so that the new elapsed time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // set process covariance matrix Q = G * Qnu * G_transpose

  // create the G matrix with the time delta
  MatrixXd G = MatrixXd::Zero(4, 2);
  G(0,0) = (dt*dt)/2.0;
  G(1,1) = (dt*dt)/2.0;
  G(2,0) = dt;
  G(3,1) = dt;

  // create the Qnu with the acceleration noise covariance
  // (noise_ax = variance of ax and noise_ay is variance of ay)
  MatrixXd Qnu = MatrixXd::Zero(2, 2);
  Qnu(0,0) = noise_ax;
  Qnu(1,1) = noise_ay;

  // set Q - the process covariance matrix
  ekf_.Q_ = G * Qnu * G.transpose();

  // call predict
  ekf_.Predict();

  /**
   * Update
   */

  // use the sensor type to perform the update step
  // update the state and covariance matrices
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates

    // set the KF H to H_jacobian, the measurement function used to map
    // radar state to measurement using multi-dimensional Taylor series
    // expansion for linear approximation
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

    // update R to radar R
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates

    // set KF H and R for Laser measurement
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
