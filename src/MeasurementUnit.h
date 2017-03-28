//
// Created by milutin on 25.3.17..
//

#ifndef EXTENDEDKF_MEASUREMENTUNIT_H
#define EXTENDEDKF_MEASUREMENTUNIT_H

#include "Eigen/Dense"


class MeasurementUnit {
  /**
   * Base class for measurement units. Each Extended Kalman Filter can use measurements from multiple sources.
   * Each measurement source must be able to update state of of the EKF and initialize its state in case there is no
   * previous knowledge about the state of the process
   *
   * This class is the base class for measurement units and defines the functions that each measurement unit must
   * implement.
   *
   * Each measurement unit must implement:
   *
   *     MeasurementFunction - it should calculate the expected measurement value for the current state of the system,
   *                           it is used for calculating measurement error
   *
   *     CheckMeasurement - used to check if provided measurement is valid
   *
   *     GetJacobian -  it should calculate jacobian for the current state. In case of linear measurement functions the
   *                    jacobian is the same as matrix H
   */
public:
  /**
  * Constructor.
  */
  MeasurementUnit(const Eigen::MatrixXd &R): R_(R){};

  /**
  * Destructor.
  */
  ~MeasurementUnit(){};

  /**
   * Used to check if provided measurement is valid
   * @param measurement
   * @return true if measurement has required shape and values
   */
  virtual bool CheckMeasurement(const Eigen::VectorXd& measurement)=0;

  /**
   * Used to update state value x and covariance P
   * @param measurement - new measurement
   * @param x - state of the process
   * @param P - current covariance of the process
   */
  void Update(const Eigen::VectorXd& measurement, Eigen::VectorXd& x, Eigen::MatrixXd& P);

  /**
   * Inverse measurement function, based on the measured variables it should return the state that produced it.
   * The state is not unique, but one has to be returned.
   * @param measurement
   * @return state that produces such provided measurement
   */
  virtual Eigen::VectorXd Inverse(const Eigen::VectorXd& measurement)=0;

protected:
  /**
   * Implementation of measurement function. Based on state of the process x it should calculate how the measurement
   * should look like.
   * @param state
   * @return expected measurement for state of the process
   */
  virtual Eigen::VectorXd  MeasurementFunction(const Eigen::VectorXd& state)=0;

  /**
   * Function that calculates the measurement error. By default the measurement error is the difference between the
   * measurement and expected measurement for a given state of the process. It doesn't need to be that way, for example
   * when calculating error in angles, it must be taken into account that difference between pi and -pi, is 0 rather than
   * 2*pi obtained by simple subtraction
   * @param measurement
   * @param state
   * @return
   */
  virtual Eigen::VectorXd  MeasurementError(const Eigen::VectorXd& measurement, const Eigen::VectorXd& state);

  /**
   * Matrix used to calculate the Jacobian matrix of the measurement based on the current state of the process.
   * Jacobian matrix is used for the update step
   *
   * @param x
   * @return
   */
  virtual Eigen::MatrixXd  GetJacobian(const Eigen::VectorXd x)=0;

  // covariance of the measurement
  Eigen::MatrixXd R_;

};


#endif //EXTENDEDKF_MEASUREMENTUNIT_H
