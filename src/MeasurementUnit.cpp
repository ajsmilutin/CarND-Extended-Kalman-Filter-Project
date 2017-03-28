//
// Created by milutin on 25.3.17..
//

#include "MeasurementUnit.h"
#include <iostream>

using namespace std;


/***
 * Implementation of update function
 */
void MeasurementUnit::Update(const Eigen::VectorXd& measurement, Eigen::VectorXd& x, Eigen::MatrixXd& P) {
  Eigen::VectorXd y = MeasurementError(measurement, x);
  Eigen::MatrixXd Hj = GetJacobian(x);
  Eigen::MatrixXd S = Hj * P * Hj.transpose() + R_;
  Eigen::MatrixXd K = P * Hj.transpose() * S.inverse();

  x+= K*y;

  P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K*Hj)*P;

}

/**
 * Default Measurement Error function, just difference between measurement and expected measurement for given state
 */
Eigen::VectorXd
MeasurementUnit::MeasurementError(const Eigen::VectorXd &measurement, const Eigen::VectorXd &state) {
  return measurement - MeasurementFunction(state);
}
