#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_aug_;

  x_.fill(0.0);
  P_ << 1, 0, 0, 0, 0,
        0, 1, 0, 0, 0,
        0, 0, 10, 0, 0,
        0, 0, 0, 1, 0,
        0, 0, 0, 0, 10;

  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  Xsig_pred_.fill(0.0);

  weights_ = VectorXd(2 * n_aug_ + 1);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    if(i == 0){
      weights_(i) = 1.0 * lambda_ / (lambda_ + n_aug_);
      continue;
    }
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  if(!is_initialized_){
    if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
      is_initialized_ = true;
      x_.head(2) = meas_package.raw_measurements_;
      x_(3) = atan2(x_(1), x_(0));

    }else if( meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
      is_initialized_ = true;
      double ro = meas_package.raw_measurements_(0);
      double yaw = meas_package.raw_measurements_(1);
      double ro_rate = meas_package.raw_measurements_(2);

      x_(0) = ro * cos(yaw);
      x_(1) = ro * sin(yaw);
      x_(2) = ro_rate;
      x_(3) = yaw;
    }

    time_us_ = meas_package.timestamp_;
    return;
  }

  // Prediction.
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;
  Prediction(delta_t);

  // Update.
  if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_){
    UpdateLidar(meas_package);
  }else if( meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_){
    UpdateRadar(meas_package);
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // Generate augument sigma points, Xsig_aug.
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.block(0, 0, n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;
  MatrixXd A_aug = P_aug.llt().matrixL();

  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0)  = x_aug;
  Xsig_aug.block(0, 1, n_aug_, n_aug_) = x_aug.replicate(1, n_aug_) + sqrt(lambda_ + n_aug_) * A_aug;
  Xsig_aug.block(0, 1 + n_aug_, n_aug_, n_aug_) = x_aug.replicate(1, n_aug_) - sqrt(lambda_ + n_aug_) * A_aug;

  // Sigma point prediction.
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    Xsig_pred_.col(i) = Xsig_aug.col(i).head(n_x_);

    double v = Xsig_aug.col(i)(2);
    double yaw = Xsig_aug.col(i)(3);
    double yaw_rate = Xsig_aug.col(i)(4);
    double std_a = Xsig_aug.col(i)(5);
    double std_yawdd = Xsig_aug.col(i)(6);

    VectorXd process_noise (n_x_);
    process_noise <<    0.5 * delta_t * delta_t * cos(yaw) * std_a,
                        0.5 * delta_t * delta_t * sin(yaw) * std_a,
                        delta_t * std_a,
                        0.5 * delta_t * delta_t * std_yawdd,
                        delta_t * std_yawdd;
    Xsig_pred_.col(i) += process_noise;

    VectorXd process(n_x_);
    if(fabs(yaw_rate) < 0.0001){
      process <<  v * cos(yaw) * delta_t,
                  v * sin(yaw) * delta_t,
                  0,
                  0,
                  0;
    }else{
      process <<  v * (sin(yaw + yaw_rate * delta_t) - sin(yaw)) / yaw_rate,
                  v * (-cos(yaw + yaw_rate * delta_t) + cos(yaw)) / yaw_rate,
                  0,
                  yaw_rate * delta_t,
                  0;
    }
    Xsig_pred_.col(i) += process;
  }

  // Calculate Predicted Mean and Covariance.
  x_ = (weights_.transpose().replicate(n_x_, 1).array() * Xsig_pred_.array()).rowwise().sum();

  P_.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd diff = Xsig_pred_.col(i) - x_;

    while(diff(3) > M_PI)   diff(3)-=2.*M_PI;
    while(diff(3) < -M_PI)  diff(3)+=2.*M_PI;

    P_ += weights_(i) * diff * diff.transpose();
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  MatrixXd Zsig (2, 2 * n_aug_ + 1);
  Zsig = Xsig_pred_.block(0, 0, 2, 2 * n_aug_ + 1);
  VectorXd z_pred = (weights_.transpose().replicate(2,1).array() * Zsig.array()).rowwise().sum();

  MatrixXd R (2, 2);
  R <<  std_laspx_ * std_laspx_, 0,
        0, std_laspy_ * std_laspy_;

  MatrixXd S = R;
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd diff = Zsig.col(i) - z_pred;
    S += weights_(i) * diff * (diff.transpose());
  }

  MatrixXd T (5, 2);
  T.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    T += weights_(i) * (Xsig_pred_.col(i) - x_) * ((Zsig.col(i) - z_pred).transpose());
  }

  MatrixXd K = T * S.inverse();
  x_ = x_ + K * (meas_package.raw_measurements_- z_pred);
  P_ = P_ - K * S * K.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  MatrixXd Zsig (3, 2 * n_aug_ + 1);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd x = Xsig_pred_.col(i);
    double px = x(0);
    double py = x(1);
    double v= x(2);
    double yaw = x(3);
    Zsig.col(i) << sqrt(px * px + py * py),
                atan2(py, px),
                (px * cos(yaw) * v + py * sin(yaw) * v) / sqrt(px * px + py * py);
  }

  VectorXd z_pred(3);
  z_pred = (weights_.transpose().replicate(3, 1).array() * Zsig.array()).rowwise().sum();

  MatrixXd R (3, 3);
  R <<  std_radr_ * std_radr_, 0, 0,
        0, std_radphi_ * std_radphi_, 0,
        0, 0, std_radrd_ * std_radrd_;

  MatrixXd S = R;
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    VectorXd diff = Zsig.col(i) - z_pred;
    S += weights_(i) * diff * (diff.transpose());
  }

  MatrixXd T (5, 3);
  T.fill(0.0);
  for(int i = 0; i < 2 * n_aug_ + 1; ++i){
    T += weights_(i) * (Xsig_pred_.col(i) - x_) * (( Zsig.col(i)- z_pred).transpose());
  }

  MatrixXd K = T * S.inverse();
  x_ = x_ + K * (meas_package.raw_measurements_ - z_pred);
  P_ = P_ - K * S * K.transpose();
}
