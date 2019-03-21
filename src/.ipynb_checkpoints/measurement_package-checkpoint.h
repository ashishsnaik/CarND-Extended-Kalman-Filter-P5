#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

/**
 * Definition for sensor measurements
 */
class MeasurementPackage {
 public:
  // type of sensor measurement
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  // timestamp of the measurement
  long long timestamp_;

  // raw sensor measurement
  Eigen::VectorXd raw_measurements_;
};

#endif // MEASUREMENT_PACKAGE_H_
