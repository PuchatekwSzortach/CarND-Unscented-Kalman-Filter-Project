#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

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

  // Initialized weights matrix to some vector.
  // Exact dimension will depend on type of sensor we are using at given update
  this->weights_ = VectorXd(15) ;

  // We will use a CTRV model and will be predicting 5 variables:
  // x and y positions, longitudinal speed, yaw and yaw rate
  this->n_x_ = 5 ;

  // Augmented state has two additional dimensions due to longitudinal and yaw accelerations
  this->n_aug_ = this->n_x_ + 2 ;

  // Per lecture notes recommendations
  this->lambda_ = 3 - this->n_x_ ;

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

    // After initialization return, we won't be doing prediction and update cycle on initialization
    return ;

  }

  //create sigma point matrix
  MatrixXd sigma_points_matrix = this->getSigmaPointsMatrix();
  std::cout << "Sigma points matrix\n" << sigma_points_matrix << std::endl ;

  if(meas_package.sensor_type_ == MeasurementPackage::LASER) {

    if(!this->use_laser_)
    {
      return ;
    }

    // Predict new state with knowledge from previous measurement
    float time_delta = (meas_package.timestamp_ - this->time_us_) / 1000000.0 ;
    this->time_us_ = meas_package.timestamp_ ;


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

MatrixXd UKF::getSigmaPointsMatrix()
{
  //create sigma point matrix
  MatrixXd sigma_points_matrix = MatrixXd(this->n_x_, 2 * this->n_x_ + 1) ;

  sigma_points_matrix.col(0) = this->x_ ;

  //calculate square root of P
  MatrixXd P_root = this->P_.llt().matrixL();

  double lambda_scaling = std::sqrt(this->lambda_ + this->n_x_) ;

  for(int index = 0 ; index < this->n_x_ ; ++index)
  {
    sigma_points_matrix.col(index + 1) = this->x_ + (lambda_scaling * P_root.col(index)) ;
    sigma_points_matrix.col(index + this->n_x_ + 1) = this->x_ - (lambda_scaling * P_root.col(index)) ;

  }

  return sigma_points_matrix ;

}