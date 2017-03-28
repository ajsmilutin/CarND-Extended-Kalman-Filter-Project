//
// Created by milutin on 26.3.17..
//

#ifndef EXTENDEDKF_LINEARMEASUREMENTUNIT_H
#define EXTENDEDKF_LINEARMEASUREMENTUNIT_H

#include "MeasurementUnit.h"
#include "Eigen/Dense"

class LinearMeasurementUnit: public  MeasurementUnit{
  /**
   * LinearMeasurementUnit subclasses MeasurementUnit and it represents all measurement units with linear measurement
   * function. One such measurement unit is Lidar
   * For linear measurement units following holds:
   *
   *   measurement = H_ * process_state
   */

public:
  /**
   * Constructor
   * @param H_ - H_ matrix
   * @param R_ - measurement covariance matrix
   */
  LinearMeasurementUnit(const Eigen::MatrixXd& H, const Eigen::MatrixXd& R_): H_(H), MeasurementUnit(R_){};

  /**
   * CheckMeasurement function override
   */

  virtual bool CheckMeasurement(const Eigen::VectorXd& measurement);

  /**
   *  Inverse function override
   */
  virtual Eigen::VectorXd Inverse(const Eigen::VectorXd& measurement);

protected:

  /**
   * MeasurementFunction function override
   */
  virtual Eigen::VectorXd  MeasurementFunction(const Eigen::VectorXd& state);

  /**
   * GetJacobian function override
   */
  virtual Eigen::MatrixXd  GetJacobian(const Eigen::VectorXd x);

  // H_ matrix
  Eigen::MatrixXd H_;
};


#endif //EXTENDEDKF_LINEARMEASUREMENTUNIT_H
