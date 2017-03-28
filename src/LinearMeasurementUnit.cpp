//
// Created by milutin on 26.3.17..
//

#include "LinearMeasurementUnit.h"
#include <iostream>

/**
 * MeasurementFunction function implementation
 *
 * For linear measurement units, measurement = H_ * state
 */
Eigen::VectorXd  LinearMeasurementUnit::MeasurementFunction(const Eigen::VectorXd& state){

  return H_*state;
}

/**
 * CheckMeasurement function implementation
 *
 * Only condition is that the lenght of the measurement has to be the same as the number of rows in matrix H_
 */

bool LinearMeasurementUnit::CheckMeasurement(const Eigen::VectorXd &measurement) {
  if (measurement.size() != H_.rows()) return false;
  return true;
}

/**
 * GetJacobian function implementation
 *
 * The jacobian is the same as matrix H_
 */

Eigen::MatrixXd LinearMeasurementUnit::GetJacobian(const Eigen::VectorXd x) {
  return H_;
}

/**
 * Inverse function implementation
 *
 * The result depends on the size of matrix H_
 *
 */

Eigen::VectorXd LinearMeasurementUnit::Inverse(const Eigen::VectorXd &measurement) {

  Eigen::MatrixXd inv;
  if (H_.rows()>H_.cols())
    // apply the least squares fitting to
    // H*x = m

    // the result is:
    // x = inverse(HT'*H)*HT*m
    inv = (H_.transpose()*H_).inverse()*H_.transpose();

  else if (H_.rows()<H_.cols())
   // minimize length of the solution
    //      minimize x
    //      such that H*x=m

    // solution is
    //    x = HT*inverse( H*HT)*m
    inv = H_.transpose() * (H_ * H_.transpose()).inverse();

  else
    inv=H_.inverse();

  return inv*measurement;

}
