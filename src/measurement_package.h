// ============================================================================
//  
//  Project 5:   Unscented Kalman Filter (Udacity Sensor Fusion Nanodegree)
// 
//  Authors:     Andreas Albrecht using code base/skeleton provided by Udacity
// 
//  Source:      https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git
// 
//  			 Original source authored by Aaron Brown (Udacity)
//
// ============================================================================

// Declaration of measurement sensor package class with Lidar and Radar sensors

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"


/**
 * Declare a measurement package class capable of holding either a Lidar or a 
 * Radar measurement for each simulation time step
 */
class MeasurementPackage {
public:
  long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */