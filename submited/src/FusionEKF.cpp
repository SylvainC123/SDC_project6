#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
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

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  Hj_ <<  0,0,0,0,
          0,0,0,0,
          0,0,0,0;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
                       
  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1000, 0,
		 0, 0, 0, 1000;
  
  float noise_ax = 9;
  float noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    
    // Initialize the state ekf_.x_ with the first measurement.

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ <<  1,1,1,1;

    // init Q matrix
    ekf_.Q_ = MatrixXd(4, 4);
    

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      
      // RADAR init
      float ro     = measurement_pack.raw_measurements_(0);
      float phi    = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);
      float x =ro * cos(phi);
	  float y =ro * sin(phi);
	  float vx =ro_dot * cos(phi);
	  float vy =ro_dot * sin(phi);
      ekf_.x_ << x, y, vx, vy;
      } 
      
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      // LIDAR init
      float x = measurement_pack.raw_measurements_(0);
      float y = measurement_pack.raw_measurements_(1);
      ekf_.x_ << x,y,0.0,0.0;
      }

    // time init
    previous_timestamp_ = measurement_pack.timestamp_;

    is_initialized_ = true;
    
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  // Updates the state transition matrix F according to the new elapsed time.

  float dt =  (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.F_ <<  1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1,  0,
              0, 0, 0,  1;

  ekf_.Q_ <<  dt_4/4*noise_ax,   0,               dt_3/2*noise_ax, 0,
              0,                 dt_4/4*noise_ay, 0,               dt_3/2*noise_ay,
              dt_3/2*noise_ax,   0,               dt_2*noise_ax,   0,
              0,                 dt_3/2*noise_ay, 0,               dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // Uses the sensor type to perform the update step.

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Tools tools;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
