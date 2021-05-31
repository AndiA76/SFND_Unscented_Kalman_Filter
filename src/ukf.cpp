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

// Definitions of Unscented Kalman Filter class member functions

// Goal: Predicted coordinates px, py, vx, vy of all target vehicles must have
// an RMSE <= [0.30, 0.16, 0.95, 0.70] after running for longer than 1 second.

#include "ukf.h"
#include "Eigen/Dense"

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define DEBUG 0 // Toggle debugging (0) and normal mode (1)


/**
 * Constructor: Initializes a new Unscented Kalman Filter instance.
 */
UKF::UKF()
{
  // Initialization flag: set to true after first call of ProcessMeasurement (default: false)
  is_initialized_ = false;

  // If this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // If this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //std_a_ = 30.0; // initial value => far too large => fails RMSE criterion
  //std_a_ = 5.0; // too large => fails RMSE criterion (too aggressive setting)
  //std_a_ = 3.0; // too large => fails RMSE criterion (too aggressive setting)
  std_a_ = 2.0;
  //std_a_ = 1.5;
  //std_a_ = 1.0;
  //std_a_ = 0.9;
  //std_a_ = 0.8;
  //std_a_ = 0.7;
  //std_a_ = 0.6;
  //std_a_ = 0.5;
  //std_a_ = 0.3; // => too low => fails RMSE criterion (definsive setting, slower transients)
  std_a_square_ = std_a_ * std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  //std_yawdd_ = 30.0; // initial value => far too large => fails RMSE criterion
  //std_yawdd_ = 5.0; // too large => fails RMSE criterion (too aggressive setting)
  //std_yawdd_ = 3.0; // too large => fails RMSE criterion (too aggressive setting)
  std_yawdd_ = 2.0;
  //std_yawdd_ = 1.5;
  //std_yawdd_ = 1.0;
  //std_yawdd_ = 0.9;
  //std_yawdd_ = 0.8;
  //std_yawdd_ = 0.7;
  //std_yawdd_ = 0.6;
  //std_yawdd_ = 0.55;
  //std_yawdd_ = 0.5;
  //std_yawdd_ = 0.3; // => too low => fails RMSE criterion (definsive setting, slower transients))
  std_yawdd_square_ = std_yawdd_ * std_yawdd_;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position x in m (cartesian coordinates)
  std_laspx_ = 0.15; // fixed measurement noise value given by the sensor manufacturer

  // Laser measurement noise standard deviation position y in m (cartesian coordinates)
  std_laspy_ = 0.15; // fixed measurement noise value given by the sensor manufacturer

  // Radar measurement noise standard deviation radius in m (polar coordinates)
  std_radr_ = 0.3; // fixed measurement noise value given by the sensor manufacturer

  // Radar measurement noise standard deviation angle in rad (polar coordinates)
  std_radphi_ = 0.03; // fixed measurement noise value given by the sensor manufacturer

  // Radar measurement noise standard deviation radius change in m/s (polar coordinates)
  std_radrd_ = 0.3; // fixed measurement noise value given by the sensor manufacturer
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */

  // Dimension of the original state vector using contant turn rate constant velocity model (CTRV)
  n_x_ = 5;

  // Initial state vector x_ = [px py v yaw_angle yaw_rate]
  x_ = VectorXd::Zero(n_x_);

  // Initial state covariance matrix
  P_ = MatrixXd::Identity(n_x_, n_x_);

  // Dimension of the augmented state vector
  n_x_aug_ = 7;

  // Number of sigmal points
  n_sig_ = 2 * n_x_aug_ + 1;

  // Sigma point spreading parameter (best practice setting: lambda_ = 3 - n_x_)
  lambda_ = 3 - n_x_;

  // Sigma point spread
  sig_spread_ = sqrt(lambda_ + n_x_aug_);

  // Initial sigma points matrix
  Xsig_pred_ = MatrixXd::Zero(n_x_, n_sig_);

  // Set initial weights of sigma points (only once)
  weights_ = VectorXd::Zero(n_sig_);
  double denominator = lambda_ + n_x_aug_;
  weights_(0) = lambda_ / denominator; // set first weight
  for (int i = 1; i < n_sig_; ++i)
  { // Set the rest of the n_sig_ = 2 * n_x_aug_ + 1 weights
    double weight = 0.5 / denominator;
    weights_(i) = weight;
  }

  // Initialize augmented state vector
  x_aug_ = VectorXd::Zero(n_x_aug_);

  // Initialize augmeted state covariance matrix
  P_aug_ = MatrixXd::Identity(n_x_aug_, n_x_aug_);

  // Initialize augmented sigma point matrix
  Xsig_aug_ = MatrixXd::Zero(n_x_aug_, n_sig_);

  // Initial Lidar measurement noise covariance matrix
  R_lidar_ = MatrixXd::Zero(2,2);
  R_lidar_ << std_laspx_ * std_laspx_, 0,
              0, std_laspy_ * std_laspy_;
  
  // Initial Radar measurement noise covariance matrix
  R_radar_ = MatrixXd::Zero(3, 3);
  R_radar_ << std_radr_ * std_radr_, 0, 0,
              0, std_radphi_ * std_radphi_, 0,
              0, 0, std_radrd_ * std_radrd_;
  
  // Initial Normalized Innovation Squared (NIS) value for Lidar
  NIS_lidar_ = 0.0;
  
  // Initial Normalized Innovation Squared (NIS) for Radar
  NIS_radar_ = 0.0;
}


/** 
 * Desctructor: Delete the current Unscented Kalman Filter instance.
 */
UKF::~UKF() {}


/**
 * Initialize the state vector of the Unscented Kalman Filter.
 * @param meas_package The latest measurement data of either Radar or Lidar
 */
void UKF::InitializeState(MeasurementPackage meas_package)
{
  // Initialize Kalman Filter state vector using either Lidar or Radar measurement
  if (!use_laser_ && !use_radar_)
  {
    std::cout << "Kalman Filter initialization failed" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Error: Neither Lidar nor Radar are activated!" << std::endl;
    std::cout << "Please activate Lidar or Radar or both!" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    // Initialize state vector components using a Lidar measurement (given in cartesian coordinates)
    double px   = meas_package.raw_measurements_[0];  // x-position relative to the ego car
    double py   = meas_package.raw_measurements_[1];  // y-position relative to the ego car
    double v    = 0.0;                                // velocity relative to the ego car
    double yaw  = 0.0;                                // yaw angle relative to the ego car
    double yawd = 0.0;                                // yaw rate relative to the ego car

    // Initialize state vector x_ (set to zero by default) using a Lidar measurement if available
    x_ << px, py, v, yaw, yawd;

#if DEBUG
    std::cout << "Initial Kalman Filter state vector using Lidar measurement" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "px    = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
    std::cout << "py    = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
    std::cout << "v     = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
    std::cout << "yaw   = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
    std::cout << "yawd  = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "time  = " << time_us_ << " [µs]" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    // Get Radar measurement (given in polar coordinates)
    double rho     = meas_package.raw_measurements_[0]; // range (radial distance) between target and origin
    double phi     = meas_package.raw_measurements_[1]; // bearing angle between target and origin
    // Remark: With rho_dot we only know the relative velocity component in bearing direction!

    // Initialize state vector components using a Radar measurement (converted to cartesian coordinates)
    double px   = rho * cos(phi);                       // x-position relative to the ego car
    double py   = rho * sin(phi);                       // y-position relative to the ego car
    double v    = 0.0;                                  // longitudinal velocity
    double yaw  = 0.0;                                  // yaw angle
    double yawd = 0.0;                                  // yaw rate

    // Initialize state vector x_ (set to zero by default) in cartesian coordinates using Radar measurement if available
    x_ << px, py, v, yaw, yawd;

#if DEBUG
    std::cout << "Initial Kalman Filter state vector using Radar measurement" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "px    = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
    std::cout << "py    = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
    std::cout << "v     = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
    std::cout << "yaw   = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
    std::cout << "yawd  = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "time  = " << time_us_ << " [µs]" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif
  }
  else
  {
    std::cout << "Kalman Filter initialization using a measurement failed" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Error: invalid measurement" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
  }

  // Initialize time stamp in microseconds [us] using the first valid measurment
  time_us_ = meas_package.timestamp_;

  // set initialization flag to true
  is_initialized_ = true;
}


/**
 * Unscented Kalman Filter Prediction and Measurement Process.
 * @param meas_package The latest measurement data of either radar or laser
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
  /**
   * Measurement process of the Unscented Kalman Filter switching between
   * Lidar and Radar measurments.
   * 
   * Remark: All states and measurements are measured relative to the ego car
   */
  
  // Check if Kalman Filter state has been initialized
  if (!is_initialized_)
  {
    // Initialize Kalman Filter state
    InitializeState(meas_package);

    return;
  } // is_initialized_

  /* Kalman Filter prediction step:
  *  - update the time step
  *  - update the state transition matrix F according to the next time step
  *  - update the process noise covariance matrix P
  */
  
  // Calculate the time step between the current and the previous time stamp
  double delta_t = static_cast<double>((meas_package.timestamp_ - time_us_) * 1e-6);
  
  // Update the time stamp
  time_us_ = meas_package.timestamp_;

  /**
  * Predict next Kalman Filter state at the next time step:
  * When applying Euler method for predicting large time steps, we assume that the state 
  * derivative is constant over the whole time interval. However, this leads to errors in
  * state propagation. In order to mitigate error propagation, we divide the one big step
  * into several small ones. See detail discussion @ ...
  * https://discussions.udacity.com/t/numerical-instability-of-the-implementation/230449/55
  */
  while (delta_t > 0.1)
  {
    const double dt = 0.05;
    Prediction(dt);
    delta_t -= dt;
  }
  Prediction(delta_t);

#if DEBUG
  std::cout << std::endl;
  std::cout << "Predict next Kalman Filter state vector" << std::endl;
  std::cout << "------------------------------------------------------------------" << std::endl;
  std::cout << "px      = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
  std::cout << "py      = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
  std::cout << "v       = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
  std::cout << "yaw     = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
  std::cout << "yawd    = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
  std::cout << std::endl;
  std::cout << "time    = " << time_us_ << " [µs]" << std::endl;
  std::cout << "delta_t = " << delta_t << " [µs]" << std::endl;
  std::cout << "------------------------------------------------------------------" << std::endl;
#endif

  /* Kalman Filter measurement update step:
  *  - update the Kalman filter state using either Lidar or Radar measurement
  *  - update the state and measurement noise covariance matrices
  */

  // Measurement update using either Lidar or Radar
  if (!use_laser_ && !use_radar_)
  {
    std::cout << "Kalman Filter measurement update failed" << x_ << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Error: Neither Lidar nor Radar are activated!" << std::endl;
    std::cout << "Please activate Lidar or Radar or both!" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
  }
  else if (use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER))
  {
    // Measurement update using Lidar
    UpdateLidar(meas_package);

#if DEBUG
    std::cout << "Kalman Filter measurement update using Lidar" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "px   = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
    std::cout << "py   = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
    std::cout << "v    = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
    std::cout << "yaw  = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
    std::cout << "yawd = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "time = " << time_us_ << " [µs]" << std::endl;
    std::cout << std::endl;
    std::cout << "NIS  = " << std::fixed << std::setprecision(9) << NIS_lidar_ << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif
  }
  else if (use_radar_ && (meas_package.sensor_type_ == MeasurementPackage::RADAR))
  {
    // Measurement update using Radar
    UpdateRadar(meas_package);

#if DEBUG
    std::cout << "Kalman Filter measurement update using Radar" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "px   = " << std::fixed << std::setprecision(9) << x_(0) << " [m]" << std::endl;
    std::cout << "py   = " << std::fixed << std::setprecision(9) << x_(1) << " [m]" << std::endl;
    std::cout << "v    = " << std::fixed << std::setprecision(9) << x_(2) << " [m/s]" << std::endl;
    std::cout << "yaw  = " << std::fixed << std::setprecision(9) << x_(3) << " [rad]" << std::endl;
    std::cout << "yawd = " << std::fixed << std::setprecision(9) << x_(4) << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "time = " << time_us_ << " [µs]" << std::endl;
    std::cout << std::endl;
    std::cout << "NIS  = " << std::fixed << std::setprecision(9) << NIS_radar_ << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif
  }
  else
  {
    // Received an invalid measurement => wait for the next valid one
    std::cout << "Kalman Filter measurement update failed" << x_ << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Error: invalid measurement" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
  }
}


/**
 * Generate augmented sigma points Xsig_aug_
 */
void UKF::GenerateSigmaPoints()
{
  // Initialize augmented state vector
  x_aug_ = VectorXd::Zero(n_x_aug_); // n_x_aug_ = n_x_ + 2
  x_aug_.head(n_x_) = x_; // original state vector

  // Initialize augumented state covariance matrix
  P_aug_ = MatrixXd::Zero(n_x_aug_, n_x_aug_); // n_x_aug_ = n_x_ + 2
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(n_x_aug_-2, n_x_aug_-2) = std_a_square_; // covariance of linear acceleration noise
  P_aug_(n_x_aug_-1, n_x_aug_-1) = std_yawdd_square_; // covariance of angular acceleration noise
  
  // Calculate the square root matrix of the agumented process noise covariance matrix
  MatrixXd L = P_aug_.llt().matrixL(); // use Cholesky decomposition

  // Initialize augumented sigma point matrix
  Xsig_aug_ = MatrixXd::Zero(n_x_aug_, n_sig_);
  Xsig_aug_.col(0) = x_aug_; // set first column
  for (int i = 0; i < n_x_aug_; ++i)
  {
    // Fill the other columns
    Xsig_aug_.col(i + 1)            = x_aug_ + sig_spread_ * L.col(i);
    Xsig_aug_.col(i + 1 + n_x_aug_) = x_aug_ - sig_spread_ * L.col(i);

    // Yaw angle sigma point normalization modulo +/- M_PI
    while (Xsig_aug_(3, i) > M_PI) Xsig_aug_(3, i) -= M_PI_x_2_;
    while (Xsig_aug_(3, i) < -M_PI) Xsig_aug_(3, i) += M_PI_x_2_;
  }
}


/**
 * Predict augmented sigma points Xsig_pred_
 * @param delta_t Time between k and k+1 in s
 */
void UKF::PredictSigmaPoints(double delta_t)
{
  // Remark: All states are measured relative to the ego car

  // Loop over all sigma points
  for (int i = 0; i < n_sig_; ++i)
  {
    // Extract state vector elements from augmented sigma point matrix for better readability
    double px       = Xsig_aug_(0, i); // x-position relative to the ego car
    double py       = Xsig_aug_(1, i); // y-position relative to the ego car
    double v        = Xsig_aug_(2, i); // longitudinal velocity
    double yaw      = Xsig_aug_(3, i); // yaw angle [-M_PI, +M_PI]
    double yawd     = Xsig_aug_(4, i); // yaw rate
    double nu_a     = Xsig_aug_(5, i); // linear accelaration noise
    double nu_yawdd = Xsig_aug_(6, i); // angular acceleration noise

    // Declare predicted state vector comnponents
    double px_pred, py_pred, v_pred, yaw_pred, yawd_pred;

    // Precidct next velocity = const. (assumption)
    v_pred = v;

    // Predict next yaw angle
    yaw_pred = yaw + yawd * delta_t;

    // Predicted yaw angle normalization modulo +/- M_PI
    while (yaw_pred > M_PI) yaw_pred -= M_PI_x_2_;
    while (yaw_pred < -M_PI) yaw_pred += M_PI_x_2_;

    // Predict next yaw rate = const. (assumption)
    yawd_pred = yawd;

    // Predict next x-y-position using CTRV vehicle motion model (avoid division by zero)
    if (fabs(yawd) > 0.001)
    {
      // Predict next predicted position on circular path
      double v_yawd_ratio = v/yawd;
      px_pred = px + v_yawd_ratio * (sin(yaw_pred) - sin(yaw));
      py_pred = py + v_yawd_ratio * (-cos(yaw_pred) + cos(yaw));
    }
    else
    {
      // Predict next predicted position on straight path
      double delta_p = v * delta_t;
      px_pred = px + delta_p * cos(yaw);
      py_pred = py + delta_p * sin(yaw);
    }

    // Initialize process noise (given as noise on linear acceleration nu_a and noise on angular acceleration nu_yawdd)
    double delta_t_square_half = 0.5 * delta_t * delta_t;
    double nu_px   = delta_t_square_half * cos(yaw) * nu_a;
    double nu_py   = delta_t_square_half * sin(yaw) * nu_a;
    double nu_v    = delta_t * nu_a;
    double nu_yaw  = delta_t_square_half * nu_yawdd;
    double nu_yawd = delta_t * nu_yawdd;

    // Add process noise effect to predicted state vector
    px_pred   = px_pred + nu_px;
    py_pred   = py_pred + nu_py;
    v_pred    = v_pred + nu_v;
    yaw_pred  = yaw_pred + nu_yaw;
    yawd_pred = yawd_pred + nu_yawd;

    // Predicted yaw angle normalization modulo +/- M_PI
    while (yaw_pred > M_PI) yaw_pred -= M_PI_x_2_;
    while (yaw_pred < -M_PI) yaw_pred += M_PI_x_2_;

#if DEBUG
    std::cout << "Process noise effect on the predicted state vector" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "nu_px    = 0.5 * delta_t^2 * cos(yaw) * nu_a = " << std::fixed << std::setprecision(9) << nu_px << " [m]" << std::endl;
    std::cout << "nu_py    = 0.5 * delta_t^2 * cos(yaw) * nu_a = " << std::fixed << std::setprecision(9) << nu_py << " [m]" << std::endl;
    std::cout << "nu_v     = delta_t * nu_a                    = " << std::fixed << std::setprecision(9) << nu_v << " [m/s]" << std::endl;
    std::cout << "nu_yaw   = 0.5 * delta_t_square * nu_yawdd   = " << std::fixed << std::setprecision(9) << nu_yaw << " [rad]" << std::endl;
    std::cout << "nu_yawd  = delta_t * nu_yawdd                = " << std::fixed << std::setprecision(9) << nu_yawd << " [rad/s]" << std::endl;
    std::cout << std::endl;
    std::cout << "nu_a     = " << std::fixed << std::setprecision(9) << nu_a << " [m/s^2]" << std::endl;
    std::cout << "nu_yawdd = " << std::fixed << std::setprecision(9) << nu_yawdd << " [rad/s^2]" << std::endl;
    std::cout << std::endl;
    std::cout << "time     = " << std::fixed << std::setprecision(9) << time_us_ << " [µs]" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif

    // Update Xsig_pred_
    Xsig_pred_(0, i) = px_pred;
    Xsig_pred_(1, i) = py_pred;
    Xsig_pred_(2, i) = v_pred;
    Xsig_pred_(3, i) = yaw_pred;
    Xsig_pred_(4, i) = yawd_pred;
  }
}


/**
 * Predict mean and covariance of the predicted state
 */
void UKF::PredictStateMeanAndCovariance()
{
  // Remark: All states are measured relative to the ego car

  // Predict state mean
  VectorXd x_pred = VectorXd::Zero(n_x_);  
  for (int i = 0; i < n_sig_; ++i)
  {
    x_pred += weights_(i) * Xsig_pred_.col(i);
  }

  // Predicted yaw angle normalization modulo +/- M_PI
  while (x_pred(3) > M_PI) x_pred(3) -= M_PI_x_2_;
  while (x_pred(3) < -M_PI) x_pred(3) += M_PI_x_2_;

  // Predict state (process noise) covariance matrix
  MatrixXd P_pred = MatrixXd::Zero(n_x_, n_x_);
  for (int i = 0; i < n_sig_; ++i)
  {
    // State residual
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
    
    // Yaw angle residual normalization modulo +/- M_PI
    while (x_diff(3) > M_PI) x_diff(3) -= M_PI_x_2_;
    while (x_diff(3) < -M_PI) x_diff(3) += M_PI_x_2_;
    
    // Final predicted state covarinace matrix
    P_pred += weights_(i) * x_diff * x_diff.transpose();
  }
  
  // Store predicted state vector and state covariance matrix
  x_ = x_pred;
  P_ = P_pred;
}


/**
 * Prediction Predicts sigma points, the state, and the state covariance matrix
 * @param delta_t Time between k and k+1 in s
 */
void UKF::Prediction(double delta_t)
{  
  // Generate sigma points for the augmented state vector
  GenerateSigmaPoints();

  // Predict augmented sigma points for the next time step
  PredictSigmaPoints(delta_t);

  // Predicted state mean and covariance for the next time step
  PredictStateMeanAndCovariance();

#if DEBUG
    std::cout << "Prediction" << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Augmented sigma points = " << std::fixed << std::setprecision(9) << Xsig_aug_ << std::endl;
    std::cout << "Predicted sigma points = " << std::fixed << std::setprecision(9) << Xsig_pred_ << std::endl;
    std::cout << "Predicted state mean = " << std::fixed << std::setprecision(9) << x_ << std::endl;
    std::cout << "Predicted state covariance = " << std::fixed << std::setprecision(9) << P_ << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
#endif
}


/**
 * Predict and update the state and the state covariance matrix using a Lidar measurement
 * @param meas_package The measurement at k+1
 */
void UKF::UpdateLidar(MeasurementPackage meas_package)
{  
  // Set measurement dimension for Lidar (x and y point position)
  int n_z = 2;

  // Create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, n_sig_);

  // Create predicted mean measurement
  VectorXd z_pred = VectorXd::Zero(n_z);

  // Create measurement covariance matrix
  MatrixXd S = MatrixXd::Zero(n_z, n_z);

  /* ----------- PREDICT MEASUREMENTS ----------- */

  // Remark: All states are measured relative to the ego car

  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; ++i)
  {    
    // Measurement model in Cartesian coordinates
    Zsig(0, i) = Xsig_pred_(0, i); // x-position (px)
    Zsig(1, i) = Xsig_pred_(1, i); // y-position (py)
  }

  // Calculate'predicted mean measurement (n_sig_ = 2 * n_x_aug_+ 1 simga points)
  for (int i = 0; i < n_sig_; ++i)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Calculate predicted measurement covariance matrix (n_sig_ = 2 * n_x_aug_+ 1 simga points)
  for (int i = 0; i < n_sig_; ++i)
  {
    // Measurement residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Update measurement covariance matrix
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  S = S + R_lidar_;

  /* ----- UPDATE STATE MEAN AND COVARIANCE ----- */

  // Remark: All measurements are measured relative to the ego car

  // Get true received measurements
  VectorXd z = meas_package.raw_measurements_; // true received measurement

  // Create cross correlation matrix between sigma points in state space and in measurement space
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // Calculate cross correlation matrix (n_sig_ = 2 * n_x_aug_+ 1 simga points)
  for (int i = 0; i < n_sig_; ++i)
  {
    // State residual
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Yaw angle residual normalization modulo +/- M_PI
    while (x_diff(3) > M_PI) x_diff(3) -= M_PI_x_2_;
    while (x_diff(3) < -M_PI) x_diff(3) += M_PI_x_2_;

    // Measurement residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Update cross-correlation matrix between sigma points in state space and in measurement space
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Calculate Kalman Filter gain maxtrix
  MatrixXd K = Tc * S.inverse();

  // Measurement residuals
  VectorXd z_diff = z - z_pred;

  // Update state mean and state covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Yaw angle normalization modulo +/- M_PI
  while (x_(3) > M_PI) x_(3) -= M_PI_x_2_;
  while (x_(3) < -M_PI) x_(3) += M_PI_x_2_;

  // Calculate Normalized Innovation Squared (NIS) update for Lidar
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
}


/**
 * Predict and update the state and the state covariance matrix using a Radar measurement
 * @param meas_package The measurement at k+1
 */
void UKF::UpdateRadar(MeasurementPackage meas_package)
{
  // Set measurement dimension for Radar (distance, angle, velocity)
  int n_z = 3;

  // Create sigma point matrix in measurement space
  MatrixXd Zsig = MatrixXd::Zero(n_z, n_sig_);

  // Create predicted mean measurement
  VectorXd z_pred = VectorXd::Zero(n_z);

  // Create measurement covariance matrix
  MatrixXd S = MatrixXd::Zero(n_z, n_z);

  /* ----------- PREDICT MEASUREMENTS ----------- */

  // Remark: All states are measured relative to the ego car

  // Transform sigma points into measurement space
  for (int i = 0; i < n_sig_; ++i)
  {
    // Extract states
    double px   = Xsig_pred_(0, i); // x-position relative to the ego car
    double py   = Xsig_pred_(1, i); // y-position relative to the ego car
    double v    = Xsig_pred_(2, i); // longitudinal velocity
    double yaw  = Xsig_pred_(3, i); // yaw angle [-M_PI, +M_PI]
    double yawd = Xsig_pred_(4, i); // yaw rate

    // Calculate velocity components
    double vx   = cos(yaw) * v; // x-velocity relative to the ego car
    double vy   = sin(yaw) * v; // y-velocity relative to the ego car

    // Measurement model assuming all measurements are relative to the ego car
    Zsig(0, i) = sqrt(px * px + py * py);          // r
    Zsig(1, i) = atan2(py, px);                    // phi [-M_PI, +M_PI]
    Zsig(2, i) = (px * vx + py * vy) / Zsig(0, i); // r_dot
  }

  // Calculate predicted mean measurement (n_sig_ = 2 * n_x_aug_+ 1 simga points)
  for (int i = 0; i < n_sig_; ++i)
  {
    z_pred += weights_(i) * Zsig.col(i);
  }

  // Predicted bearing angle normalization modulo +/- M_PI
  while (z_pred(1) > M_PI) z_pred(1) -= M_PI_x_2_;
  while (z_pred(1) < -M_PI) z_pred(1) += M_PI_x_2_;

  // Calculate predicted measurement covariance matrix (n_sig_ = 2 * n_x_aug_+ 1 simga points)
  for (int i = 0; i < n_sig_; ++i)
  {
    // Measurement residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Bearing angle residual normalization modulo +/- M_PI
    while (z_diff(1) > M_PI) z_diff(1) -= M_PI_x_2_;
    while (z_diff(1) < -M_PI) z_diff(1) += M_PI_x_2_;

    // Update measurement covariance matrix
    S += weights_(i) * z_diff * z_diff.transpose();
  }

  // Add measurement noise covariance matrix
  S = S + R_radar_;

  /* ----- UPDATE STATE MEAN AND COVARIANCE ----- */

  // Remark: All measurements are measured relative to the ego car

  // Get true received measurements
  VectorXd z = meas_package.raw_measurements_;

  // Create cross correlation matrix
  MatrixXd Tc = MatrixXd::Zero(n_x_, n_z);

  // Calculate cross correlation matrix (n_sig_ = 2 * n_x_aug_+ 1 simga points)
  for (int i = 0; i < n_sig_; ++i)
  {
    // State residual
    VectorXd x_diff = Xsig_pred_.col(i) - x_;

    // Yaw angle residual normalization modulo +/- M_PI
    while (x_diff(3) > M_PI) x_diff(3) -= M_PI_x_2_;
    while (x_diff(3) < -M_PI) x_diff(3) += M_PI_x_2_;

    // Measurement residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // Bearing angle residual normalization modulo +/- M_PI
    while (z_diff(1) > M_PI) z_diff(1) -= M_PI_x_2_;
    while (z_diff(1) < -M_PI) z_diff(1) += M_PI_x_2_;

    // Update cross-correlation matrix between sigma points in state space and in measurement space
    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Calculate Kalman gain matrix
  MatrixXd K = Tc * S.inverse();

  // Update measurement residuals
  VectorXd z_diff = z - z_pred;

  // Bearing angle residual normalization modulo +/- M_PI
  while (z_diff(1) > M_PI) z_diff(1) -= M_PI_x_2_;
  while (z_diff(1) < -M_PI) z_diff(1) += M_PI_x_2_;

  // Update state mean and state covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

  // Yaw angle normalization modulo +/- M_PI
  while (x_(3) > M_PI) x_(3) -= M_PI_x_2_;
  while (x_(3) < -M_PI) x_(3) += M_PI_x_2_;

  // Calculate Normalized Innovation Squared (NIS) update for Radar
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
}