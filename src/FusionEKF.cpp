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

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // this changes everytime,
    // so let's just put it to zero now
    Hj_ << 0, 0, 0, 0,
           0, 0, 0, 0,
           0, 0, 0, 0;

    noise_ax_ = 10;
    noise_ay_ = 10;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    previous_timestamp_ = measurement_pack.timestamp_;

    MatrixXd F = MatrixXd(4, 4); // movement matrix
    F << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    MatrixXd P = MatrixXd(4, 4); // state cov matrix
    P << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    MatrixXd Q = MatrixXd(4, 4); // process noise
    Q << 0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;

    VectorXd x = VectorXd(4);
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        float rho = measurement_pack.raw_measurements_(0);
        float theta = measurement_pack.raw_measurements_(1);

        x << rho*cos(theta), rho*sin(theta), 0, 0;

        ekf_.Init(x, P, F, Hj_, R_radar_, Q);

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
        x << measurement_pack.raw_measurements_(0),
             measurement_pack.raw_measurements_(1), 0, 0;
        ekf_.Init(x, P, F, Hj_, R_radar_, Q);
        cout << "init with laser!" << endl;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  float dt2 = dt*dt;
  float dt3 = dt2*dt;
  float dt4 = dt3*dt;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  MatrixXd Q = MatrixXd(4, 4);
  Q << dt4*noise_ax_/4, 0, dt3*noise_ax_/2, 0,
       0, dt4*noise_ay_/4, 0, dt3*noise_ay_/2,
       dt3*noise_ax_/2, 0, dt2*noise_ax_, 0,
       0, dt3*noise_ay_/2, 0, dt2*noise_ay_;
  ekf_.Q_ = Q;

  ekf_.Predict();
  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
      Tools tool;
      ekf_.H_ = tool.CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;

      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;

      ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
