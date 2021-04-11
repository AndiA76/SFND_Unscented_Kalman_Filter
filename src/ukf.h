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

// Unscented Kalman Filter class member functions and variables

#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF
{
  public:

  /* CLASS MEMBER FUNCTIONS */

  /**
   * Constructor.
   */
  UKF();

  /**
   * Destructor.
   */
  virtual ~UKF();

  /**
   * Initialize Unscented Kalman Filter state.
   * @param meas_package The latest measurement data of either Radar or Lidar
   */
  void InitializeState(MeasurementPackage meas_package);

  /**
   * Unscented Kalman Filter Prediction and Measurement Process.
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Generate sigma points for the augmented state vector.
   */
  void GenerateSigmaPoints(); // AugmentSigmaPoints()

  /**
   * Predict sigma points for the augmented state vector.
   * @param delta_t Time between k and k+1 in s
   */
  void PredictSigmaPoints(double delta_t);

  /**
   * Predict mean and covariance of the predicted state.
   */
  void PredictStateMeanAndCovariance();

  /**
   * Predict the sigma points, the state, and the state covariance for the next time step.
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Predict and update the state and the state covariance matrix using a Lidar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Predict and update the state and the state covariance matrix using a Radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  /* STATIC CONSTANTS */

  // 2 * PI
  static constexpr double M_PI_x_2_ = 2 * M_PI;


  /* CLASS MEMBER VARIALBES */

  // Initialization flag: Initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // If this is false, Lidar measurements will be ignored (except for init)
  bool use_laser_;

  // If this is false, Radar measurements will be ignored (except for init)
  bool use_radar_;

  // Timestamp of the previous step in µs when the state is true
  long long time_us_;

  // State vector dimension
  int n_x_;

  // State vector: [x-position, y-position, longitudinal velocity, yaw angle, yaw rate] in SI units and rad
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // Number of sigma points
  int n_sig_;

  // Sigma point spreading parameter
  double lambda_;

  // Sigma point spread
  double sig_spread_;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // Predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // Augmented state dimension
  int n_x_aug_;

  // Augmented state vector
  Eigen::VectorXd x_aug_;

  // Augmented state covariance matrix
  Eigen::MatrixXd P_aug_;

  // Augmented sigma points matrix
  Eigen::MatrixXd Xsig_aug_;

  // Process noise standard deviation longitudinal acceleration in m/s^2 (cartesian coordinates)
  double std_a_;
  double std_a_square_;

  // Process noise standard deviation yaw acceleration in rad/s^2 (cartesian coordinates)
  double std_yawdd_;
  double std_yawdd_square_;

  // Laser measurement noise standard deviation position x in m (cartesian coordinates)
  double std_laspx_;

  // Laser measurement noise standard deviation position y in m (cartesian coordinates)
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m (polar coordinates)
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad (polar coordinates)
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s (polar coordinates)
  double std_radrd_ ;
  
  // Radar measurement noise covariance matrix
  Eigen::MatrixXd R_radar_;

  // Lidar measurement noise covariance matrix
  Eigen::MatrixXd R_lidar_;

  // Normalized Innovation Squared (NIS) value for Lidar
  double NIS_lidar_;
  
  // Normalized Innovation Squared (NIS) for Radar
  double NIS_radar_;
};

#endif  // UKF_H