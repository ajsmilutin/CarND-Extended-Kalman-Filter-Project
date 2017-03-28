//
// Created by milutin on 25.3.17..
//

#ifndef EXTENDEDKF_RADARMEASUREMENTUNIT_H
#define EXTENDEDKF_RADARMEASUREMENTUNIT_H

#include "MeasurementUnit.h"
#include "Eigen/Dense"
class RadarMeasurementUnit: public  MeasurementUnit{
 /**
  * RadarMeasurementUnit subclasses MeasurementUnit and it implements the way radar works
  * The class should take care of transformation of polar to cartesian coordinates and
  * vice-versa
  */


public:
  /**
   * Constructor
   * @param R_ - measurement covariance matrix
   */
  RadarMeasurementUnit(const Eigen::MatrixXd& R_): MeasurementUnit(R_){};

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
   * MeasurementError function override
   */
  Eigen::VectorXd MeasurementError(const Eigen::VectorXd &measurement, const Eigen::VectorXd &state) override;

  /**
   * MeasurementFunction function override
   */
  Eigen::VectorXd  MeasurementFunction(const Eigen::VectorXd& state );

  /**
   * GetJacobian function override
   */
  virtual Eigen::MatrixXd  GetJacobian(const Eigen::VectorXd x);
};


#endif //EXTENDEDKF_RADARMEASUREMENTUNIT_H
