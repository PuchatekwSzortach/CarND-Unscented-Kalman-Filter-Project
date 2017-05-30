#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "Tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 4;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI / 4.0;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  // We will use a CTRV model and will be predicting 5 variables:
  // x and y positions, longitudinal speed, yaw and yaw rate
  this->n_x_ = 5 ;

  // Augmented state has two additional dimensions due to longitudinal and yaw accelerations
  this->n_aug_ = this->n_x_ + 2 ;

  // Per lecture notes recommendations
  this->lambda_ = 3 - this->n_x_ ;

  this->weights_ = this->getSigmaPointsWeights() ;
  this->Xsig_pred_ = MatrixXd(this->n_x_, 2 * this->n_aug_ + 1) ;

  this->use_laser_ = true ;

  std::cout << "NOT USING RADAR FOR NOW!" << std::endl ;
  this->use_radar_ = false ;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  if(!this->is_initialized_) {

    this->initializeUKF(meas_package) ;
    this->is_initialized_ = true ;

    // After initialization return, no need to do prediction and update cycle on initialization
    return ;

  }

  if(meas_package.sensor_type_ == MeasurementPackage::LASER) {

    if(!this->use_laser_)
    {
      return ;
    }

    // Predict new state with knowledge from previous measurement
    double time_delta = (meas_package.timestamp_ - this->time_us_) / 1000000.0 ;
    this->time_us_ = meas_package.timestamp_ ;

    this->Prediction(time_delta) ;
    this->UpdateLidar(meas_package) ;

  } else
  {
    if(!this->use_radar_)
    {
      return ;
    }

    std::cout << "Radar measurement came in" << std::endl ;


  }


}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  //create sigma point matrix
  MatrixXd sigma_points = this->getSigmaPoints();
  MatrixXd augmented_sigma_points = this->getAugmentedSigmaPoints(sigma_points) ;

  // Create sigma predictions, compute mean predictions and prediction covariance matrix
  this->Xsig_pred_ = this->getSigmaPointsPredictions(augmented_sigma_points, delta_t) ;
  this->x_ = this->getMeanPrediction(this->Xsig_pred_, this->n_x_) ;
  this->P_ = this->getPredictionCovarianceMatrix(this->Xsig_pred_, this->x_) ;

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

  std::cout << "Update Lidar called" << std::endl ;

  // Laser measurements have two dimensions, px and py
  int n_z = 2 ;

  MatrixXd laser_measurements_predictions = this->getLaserMeasurementsPredictions(this->Xsig_pred_) ;
  VectorXd mean_measurement_prediction = this->getMeanPrediction(laser_measurements_predictions, n_z) ;

  MatrixXd laser_measurement_prediction_covariane_matrix = this->getLaserMeasurementPredictionCovarianceMatrix(
    laser_measurements_predictions, mean_measurement_prediction) ;

  MatrixXd cross_correlation_matrix = this->getLaserCrossCorrelationMatrix(
    laser_measurements_predictions, mean_measurement_prediction, n_z) ;

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}


void UKF::initializeUKF(MeasurementPackage meas_package)
{
  if(meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    // x and y positions, known from laser
    this->x_(0) = meas_package.raw_measurements_(0) ;
    this->x_(1) = meas_package.raw_measurements_(1) ;

    // Velocity, yaw and yaw rate are unknown, set them to zero
    this->x_(2) = 0 ;
    this->x_(3) = 0 ;
    this->x_(4) = 0 ;

    this->time_us_ = meas_package.timestamp_ ;
    this->is_initialized_ = true ;

  } else
  {
    std::cout << "RADAR measurement to initialize with, we don't handle that yet" << std::endl ;
    exit(0) ;
  }

  // Initialize covariance matrix P
  this->P_.setIdentity(5, 5) ;
}

MatrixXd UKF::getSigmaPoints()
{
  //create sigma point matrix
  MatrixXd sigma_points = MatrixXd(this->n_x_, 2 * this->n_x_ + 1) ;

  sigma_points.col(0) = this->x_ ;

  //calculate square root of P
  MatrixXd P_root = this->P_.llt().matrixL();

  double lambda_scaling = std::sqrt(this->lambda_ + this->n_x_) ;

  for(int index = 0 ; index < this->n_x_ ; ++index)
  {
    sigma_points.col(index + 1) = this->x_ + (lambda_scaling * P_root.col(index)) ;
    sigma_points.col(index + this->n_x_ + 1) = this->x_ - (lambda_scaling * P_root.col(index)) ;

  }

  return sigma_points ;

}

MatrixXd UKF::getAugmentedSigmaPoints(MatrixXd sigma_points)
{
  // Create augmented vector
  VectorXd x_augmented = VectorXd(this->n_aug_) ;
  x_augmented.fill(0) ;
  x_augmented.head(5) = this->x_ ;

  // Create augmented covariance matrix
  MatrixXd P_augmented = MatrixXd(this->n_aug_, this->n_aug_) ;
  P_augmented.fill(0) ;
  P_augmented.topLeftCorner(this->n_x_, this->n_x_) = this->P_ ;
  P_augmented(5, 5) = this->std_a_ * this->std_a_ ;
  P_augmented(6, 6) = this->std_yawdd_ * this->std_yawdd_ ;

  //create augmented square root matrix
  MatrixXd P_augmented_root = P_augmented.llt().matrixL();

  MatrixXd augmented_sigma_points = MatrixXd(this->n_aug_, 2 * this->n_aug_ + 1) ;
  augmented_sigma_points.col(0) = x_augmented ;

  double lambda_scaling = std::sqrt(this->lambda_ + this->n_aug_) ;

  for(int index = 0 ; index < this->n_aug_ ; ++index)
  {
    augmented_sigma_points.col(index + 1) =
      x_augmented + (lambda_scaling * P_augmented_root.col(index));

    augmented_sigma_points.col(index + this->n_aug_ + 1) =
      x_augmented - (lambda_scaling * P_augmented_root.col(index));
  }

  return augmented_sigma_points ;
}

MatrixXd UKF::getSigmaPointsPredictions(MatrixXd augmented_sigma_points, double time_delta)
{
  MatrixXd predictions = MatrixXd(this->n_x_, 2 * this->n_aug_ + 1) ;

  for(int index = 0 ; index < 2 * this->n_aug_ + 1 ; ++index)
  {
    VectorXd current_state = augmented_sigma_points.col(index).head(this->n_x_) ;

    float longitudinal_speed = current_state(2) ;
    float yaw = current_state(3) ;
    float yaw_speed = current_state(4) ;

    float random_linear_acceleration = augmented_sigma_points(5, index) ;
    float random_yaw_acceleration = augmented_sigma_points(6, index) ;

    float speed_ratios = longitudinal_speed / yaw_speed ;
    float interpolated_yaw = yaw + (time_delta * yaw_speed) ;

    VectorXd transition_vector = VectorXd(5) ;

    if(std::abs(yaw_speed) < 0.001)
    {
      transition_vector(0) = time_delta * longitudinal_speed * std::cos(yaw) ;
      transition_vector(1) = time_delta * longitudinal_speed * std::sin(yaw) ;

    }
    else
    {
      transition_vector(0) = speed_ratios * (std::sin(interpolated_yaw) - std::sin(yaw));
      transition_vector(1) = speed_ratios * (-std::cos(interpolated_yaw) + std::cos(yaw));
    }

    transition_vector(3) = yaw_speed * time_delta ;

    VectorXd noise_vector = VectorXd(5) ;
    double half_squared_time_delta = 0.5 * time_delta * time_delta ;

    noise_vector(0) = half_squared_time_delta * std::cos(yaw) * random_linear_acceleration ;
    noise_vector(1) = half_squared_time_delta * std::sin(yaw) * random_linear_acceleration ;
    noise_vector(2) = time_delta * random_linear_acceleration ;
    noise_vector(3) = half_squared_time_delta * random_yaw_acceleration ;
    noise_vector(4) = time_delta * random_yaw_acceleration ;

    predictions.col(index) = current_state + transition_vector + noise_vector ;

  }

  return predictions ;
}

VectorXd UKF::getSigmaPointsWeights()
{
  VectorXd weights = VectorXd(2 * this->n_aug_ + 1);

  weights(0) = this->lambda_ / (this->lambda_ + this->n_aug_) ;

  for(int index = 1 ; index < 2 * this->n_aug_ + 1 ; ++index)
  {
    weights(index) = 0.5 / (this->lambda_ + this->n_aug_) ;
  }

  return weights ;
}

VectorXd UKF::getMeanPrediction(MatrixXd sigma_points_predictions, int dimensions)
{
  VectorXd mean_prediction = VectorXd(dimensions);
  mean_prediction.fill(0);

  for(int index = 0 ; index <  2 * this->n_aug_ + 1 ; ++index)
  {
    mean_prediction += this->weights_(index) * sigma_points_predictions.col(index) ;
  }

  return mean_prediction ;
}

MatrixXd UKF::getPredictionCovarianceMatrix(MatrixXd sigma_points_predictions, VectorXd mean_prediction)
{
  MatrixXd covariance_matrix = MatrixXd(this->n_x_, this->n_x_);
  covariance_matrix.fill(0) ;

  for(int index = 0 ; index < 2 * this->n_aug_ + 1 ; ++index)
  {
    VectorXd difference = sigma_points_predictions.col(index) - mean_prediction ;

    // Normalize yaw angle to <-PI, PI> interval
    difference(3) = Tools().getNormalizedAngle(difference(3)) ;

    covariance_matrix += this->weights_(index) * difference * difference.transpose() ;
  }

  return covariance_matrix ;
}

MatrixXd UKF::getLaserMeasurementsPredictions(MatrixXd sigma_points_predictions)
{
  MatrixXd laser_measurements_predictions = MatrixXd(2, 2 * this->n_aug_ + 1) ;

  for(int index = 0 ; index < 2 * this->n_aug_ + 1 ; ++index)
  {
    // For laser measurements we just copy prediction px and py
    laser_measurements_predictions(0, index) = sigma_points_predictions(0, index) ;
    laser_measurements_predictions(1, index) = sigma_points_predictions(1, index) ;
  }

  return laser_measurements_predictions ;

}

MatrixXd UKF::getLaserMeasurementPredictionCovarianceMatrix(
  MatrixXd laser_measurements_predictions, VectorXd mean_measurement_prediction)
{

  MatrixXd covariane_matrix = MatrixXd(2, 2);

  for(int index = 0 ; index < 2 * this->n_aug_ + 1 ; ++index)
  {
    VectorXd difference = laser_measurements_predictions.col(index) - mean_measurement_prediction ;

    covariane_matrix += this->weights_(index) * difference * difference.transpose() ;
  }

  MatrixXd measurement_noise_covariance_matrix(2, 2) ;
  measurement_noise_covariance_matrix.fill(0) ;
  measurement_noise_covariance_matrix(0, 0) = this->std_laspx_ * this->std_laspx_ ;
  measurement_noise_covariance_matrix(1, 1) = this->std_laspy_ * this->std_laspy_ ;

  covariane_matrix += measurement_noise_covariance_matrix ;

  return covariane_matrix ;
}

MatrixXd UKF::getLaserCrossCorrelationMatrix(
  MatrixXd measurements_predictions, VectorXd measurement_prediction, int measurement_dimensions)
{

  MatrixXd cross_correlation_matrix = MatrixXd(this->n_x_, measurement_dimensions);
  cross_correlation_matrix.fill(0) ;

  for(int index = 0 ; index < 2 * this->n_aug_ + 1 ; ++index)
  {

    VectorXd state_difference = this->Xsig_pred_.col(index) - this->x_ ;
    VectorXd measurement_difference = measurements_predictions.col(index) - measurement_prediction ;

    cross_correlation_matrix += this->weights_(index) * state_difference * measurement_difference.transpose() ;

  }

  return cross_correlation_matrix ;

}