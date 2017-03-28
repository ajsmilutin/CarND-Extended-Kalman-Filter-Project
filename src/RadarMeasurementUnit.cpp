//
// Created by milutin on 25.3.17..
//

#include "RadarMeasurementUnit.h"
#include <math.h>
#include <iostream>

/**
 * MeasurementFunction function implementation
 *
 * In essence it is transformation from cartesian to polar coordinates
 */

Eigen::VectorXd  RadarMeasurementUnit::MeasurementFunction(const Eigen::VectorXd& x){
  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);


  float rho = sqrt(px*px + py*py);
  Eigen::VectorXd result(3);

  // small number (1e-8) is added to the rho in order to avoid division by zero
  // when radius is big this does not influence the result, but in case of zero
  // radius it helps avoid divison by zero, since rho>=0. This is well know trick
  // in robotics
  result<< rho,  atan2(py, px), (px*vx + py*vy)/(rho + 1e-8);
  return result;
}


/**
 * CheckMeasurement function implementation
 *
 * The lenght of the measurement has to be 3, and the radius rho has to be greater or equal
 * to zero
 *
 */
bool RadarMeasurementUnit::CheckMeasurement(const Eigen::VectorXd& measurement){
// check for length
  if (measurement.size()!=3) return false;
// rho has to be positive
  if (measurement(0)<0) return false;

  return true;
}

/**
 * GetJacobian function implementation
 *
 */


Eigen::MatrixXd RadarMeasurementUnit::GetJacobian(const Eigen::VectorXd x) {
  Eigen::MatrixXd jacobian(3, 4);

  float px = x(0);
  float py = x(1);
  float vx = x(2);
  float vy = x(3);

  // same trick for avoiding division by zero is used here as in
  // in MeasurementFunction
  // when px = 0 and py = 0 (the only case when rho = 0) the jacobian
  // would be zero matrix, so the gain K (in Update function) would be
  // also zero matrix, meaning that such measurement would not influence
  // the state of the system

  float rho2 = px*px + py*py + 1e-16;

  float rho = sqrt(rho2);
  float tmp = (vx*py - vy*px)/rho/rho2;
  jacobian<< px/rho , py/rho ,  0   ,   0   ,
      -py/rho2, px/rho2, 0   ,   0   ,
      py*tmp,  -px*tmp, px/rho, py/rho;

  return jacobian;
}

/**
 * Inverse function implementation
 *
 * It is assumed that there is only radial velocity which is measured by radar.
 * Tangential velocity (perpendicular to the radius) is assumed to be zero.
 *
 */

Eigen::VectorXd RadarMeasurementUnit::Inverse(const Eigen::VectorXd &measurement) {
  Eigen::VectorXd result = Eigen::VectorXd(4);
  double rho = measurement(0);
  double phi = measurement(1);
  double rhodot = measurement(2);
  result<< rho * cos(phi), rho*sin(phi), rhodot*cos(phi), rhodot*sin(phi);
  return  result;
}


/**
 * MeasurementError function implementation
 *
 * The second parameter is rho which is given in radians. Funcion atan2 gives angles in range from (-pi, pi]
 * When the car crosses the x axis the value of angle can jump from  pi to -pi + epsilon. By simpel subtraction
 * the error is 2*pi-epsilon although the angle changed just for epsilon. That can produce
 * undesired behavior. This function takes care that error in angles is allso in range (-pi, pi]
 *
 */

Eigen::VectorXd
RadarMeasurementUnit::MeasurementError(const Eigen::VectorXd &measurement, const Eigen::VectorXd &state) {
  Eigen::VectorXd  result = measurement - MeasurementFunction(state);
  double tmp_res = (result(1)+ M_PI)/2/M_PI;
  tmp_res -= floor(tmp_res);
  result(1) = (tmp_res * 2*M_PI) - M_PI;
  return result;
}
