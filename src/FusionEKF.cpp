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

  noise_ax_ = 9;
  noise_ay_ = 9;

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

    // first measurement
    x_ = Units_[measurement_pack.sensor_type_]->Inverse(measurement_pack.raw_measurements_);

    // assignment based on values of measurement variance matrices
    P_ = 0.1*MatrixXd::Identity(4, 4);
    P_(2, 2) = 1;
    P_(3, 3) = 1;

    previous_timestamp_ = measurement_pack.timestamp_ ;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  Predict(measurement_pack.timestamp_);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (Units_[measurement_pack.sensor_type_]->CheckMeasurement(measurement_pack.raw_measurements_))
    Units_[measurement_pack.sensor_type_]->Update(measurement_pack.raw_measurements_, x_, P_);


}

/**
 * AddMeasurementUnit function implementation
 */
void FusionEKF::AddMeasurementUnit(MeasurementUnit* new_unit, MeasurementPackage::SensorType type) {
  Units_[type] = new_unit;
}


/**
 * Prediction step
 */
void FusionEKF::Predict(long timestamp) {
  double dt = (timestamp - previous_timestamp_)/1000000.0;
  previous_timestamp_ = timestamp;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // proces state transition matrix
  MatrixXd F = MatrixXd::Identity(4, 4);
  F(0, 2) = dt;
  F(1, 3) = dt;

  // process covariance matrix
  MatrixXd Q = MatrixXd(4, 4);
  Q <<  dt_4/4*noise_ax_, 0, dt_3/2*noise_ax_, 0,
        0, dt_4/4*noise_ay_, 0, dt_3/2*noise_ay_,
        dt_3/2*noise_ax_, 0, dt_2*noise_ax_, 0,
       0, dt_3/2*noise_ay_, 0, dt_2*noise_ay_;


  // prediction step
  x_ = F * x_;
  P_ = F * P_ * F.transpose() + Q;

}