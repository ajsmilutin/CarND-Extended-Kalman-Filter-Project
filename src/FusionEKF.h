#ifndef FusionEKF_H_
#define FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <map>
#include <string>
#include <fstream>

#include "tools.h"
#include "MeasurementUnit.h"
#include "measurement_package.h"
class FusionEKF {
  /**
   * This class implements the Extended Kalman Filter that can have multiple measurement
   * sources. The process itself is implemented in this class, while measurement updates
   * are implemented in MeasurementUnit class.
   */

public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  * @param measurement_pack package containing measurement
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Add the measurement unit
  * @param new_unit - pointer to new measuring unit
  * @param type  - sensor type, used to identify the sensor in appropriate map
  *
  */
  void AddMeasurementUnit(MeasurementUnit *new_unit, MeasurementPackage::SensorType type);


  /**
   * The function which returns the internal state
   */
   Eigen::VectorXd getX(){return x_;};


private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  // noises in x and y directon
  double noise_ax_;
  double noise_ay_;

  // estimated position
  Eigen::VectorXd x_;

  // estimation variance matrix
  Eigen::MatrixXd P_;

  // measurement packages
  std::map<MeasurementPackage::SensorType, MeasurementUnit*>  Units_;

  /**
   * Prediction step
   * @param timestamp - timestamp of measurement
   */

  void Predict(long timestamp);


};

#endif /* FusionEKF_H_ */
