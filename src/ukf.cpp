#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
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
  std_a_ = 3; // 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 3; // 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  n_x_ = 5;										//set state dimension
  n_aug_ = 7;									//set augmented dimension
  lambda_ = 3 - n_aug_;							//define spreading parameter
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);	//allocate matrix for predicted sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);			//allocate vector for weights
  
  //calculate weights for Radar update
  float w = 1 / (2 * (lambda_ + n_aug_));
  weights_.fill(w);
  w = lambda_ / (lambda_ + n_aug_);
  weights_(0) = w;

  // For Lidar update
  H_ = MatrixXd(2, 5);
  H_ << 1, 0, 0, 0, 0,
	    0, 1, 0, 0, 0;

  R_ = MatrixXd(2, 2);
  R_ << std_laspx_* std_laspx_, 0,
	    0, std_laspy_*std_laspy_;

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
	if (((meas_package.sensor_type_ == MeasurementPackage::RADAR) && !use_radar_) ||
		((meas_package.sensor_type_ == MeasurementPackage::LASER) && !use_laser_))
	{
		/* this is need to prevent the main() routine from crashing because it tries to
		* read data from "x_" that may not have been initialized.
		*/
		if (!is_initialized_)
		{
			x_ << 1, 1, 1, 1, 1;
		}
		return;
	}

	/* Sanity check. We need to re-init when we want to switch between Datatset1
	* and Dataset2 or if we want to Restart the current Dataset
	* without re-launching the application
	*/
	if (time_us_ != 0)
	{
		if (time_us_ > meas_package.timestamp_)
		{
			is_initialized_ = 0;
		}
		else
		{
			float diff = (meas_package.timestamp_ - time_us_) / 1000000.0;
			if (diff > 1000000)
			{
				is_initialized_ = 0;
			}
		}
	}

	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized_)
	{
		/**
		TODO:
		* Initialize the state ekf_.x_ with the first measurement.
		* Create the covariance matrix.
		* Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		//step_ = 0;

		// first measurement
		cout << "UKF: " << endl;

		// init state
		x_ << 1, 1, 1, 1, 1;


		// init matrix P
		P_ << 0.2, 0, 0, 0, 0,
			0, 0.2, 0, 0, 0,
			0, 0, 0.4, 0, 0,
			0, 0, 0, 0.2, 0,
			0, 0, 0, 0, 0.2;

		time_us_ = meas_package.timestamp_;

		if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		{
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			float ro = meas_package.raw_measurements_[0];
			float phi = meas_package.raw_measurements_[1];
			float rho_dot = meas_package.raw_measurements_[2];
			float px = ro * cos(phi);
			float py = ro * sin(phi);
			x_ << px, py, rho_dot, 0, 0;
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		{
			/**
			Initialize state.
			*/
			float px = meas_package.raw_measurements_[0];
			float py = meas_package.raw_measurements_[1];
			x_ << px, py, 0, 0, 0;
		}

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/
	// 1. compute the time elapsed between the current and previous measurements
	float dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
	time_us_ = meas_package.timestamp_;

	//2. Call the Unscented Kalman Filter prediction function
	Prediction(dt);

	/*****************************************************************************
	*  Update
	****************************************************************************/
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
	{
		/*
		 * Radar updates
		 */
		UpdateRadar(meas_package);
	}
	else
	{
		/*
		 * Laser updates
		 */
		UpdateLidar(meas_package);
	}

	// print the output
	// cout << "x_ = " << ekf_.x_ << endl;
	// cout << "P_ = " << ekf_.P_ << endl;
	//step_++;
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
  /*====================================== 
   *===   1. Generate Sigma Points    ==== 
   *======================================*/
  /*
   * 1.1 create augmented mean vector
   */
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  /*
   * 1.2 create augmented state covariance
   */
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  /*
   * 1.3 create square root matrix
   */
  MatrixXd A = P_aug.llt().matrixL();

  /*
   * 1.4 create augmented sigma points matrix
   */
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++)
  {
	  Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
	  Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
  }

  /*====================================
   *===   2. Predict Sigma Points     ====
   *====================================*/
  /*
   * 2.1 calculate Predicted Sigma Points. Each column is one sigma point.
   */
  VectorXd x = VectorXd(n_aug_);
  VectorXd x_pred = VectorXd(n_x_);
  double delta_t2 = delta_t * delta_t;
  for (int i = 0; i < (2 * n_aug_ + 1); i++)
  {
	  x = Xsig_aug.col(i);
	  if (x(4) == 0.0)
	  {
		  x_pred(0) = x(0) + (x(2)*cos(x(3))*delta_t) + (0.5*delta_t2*cos(x(3))*x(5));
		  x_pred(1) = x(1) + (x(2)*sin(x(3))*delta_t) + (0.5*delta_t2*sin(x(3))*x(5));
		  x_pred(2) = x(2) + 0 + (delta_t*x(5));
		  x_pred(3) = x(3) + (x(4)*delta_t) + (0.5*delta_t2*x(6));
		  x_pred(4) = x(4) + 0 + (delta_t*x(6));
	  }
	  else
	  {
		  x_pred(0) = x(0) + (x(2) / x(4))*(sin(x(3) + x(4)*delta_t) - sin(x(3))) + (0.5*delta_t2*cos(x(3))*x(5));
		  x_pred(1) = x(1) + (x(2) / x(4))*(-cos(x(3) + x(4)*delta_t) + cos(x(3))) + (0.5*delta_t2*sin(x(3))*x(5));
		  x_pred(2) = x(2) + 0 + (delta_t*x(5));
		  x_pred(3) = x(3) + (x(4)*delta_t) + (0.5*delta_t2*x(6));
		  x_pred(4) = x(4) + 0 + (delta_t*x(6));
	  }
	  Xsig_pred_.col(i) = x_pred;
  }

  /*============================================
  *===    3. Predict Mean and Covariance    ====
  *============================================*/
  /*
   * 3.1 calculate predict state mean
   */
  x_.fill(0.0);
  for (int i = 0; i < 1 + (2 * n_aug_); i++)
  {
	  x_ = x_ + weights_(i)*Xsig_pred_.col(i);
  }

  /*
   * 3.2 calculate predict state covariance matrix
   */
  VectorXd tmp;
  P_.fill(0.0);
  for (int i = 0; i < 1 + (2 * n_aug_); i++)
  {
	  tmp = (Xsig_pred_.col(i) - x_);
	  while (tmp(3)> M_PI) tmp(3) -= 2.*M_PI;
	  while (tmp(3)<-M_PI) tmp(3) += 2.*M_PI;
	  P_ += weights_(i) * tmp * tmp.transpose();
  }

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
	int n_z = 2;
	VectorXd z = VectorXd(n_z);
	z(0) = meas_package.raw_measurements_(0);
	z(1) = meas_package.raw_measurements_(1);

	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
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

  /*====================================
   *===   1.  Predict Measurement    ===
   *===================================*/
  /*
   * 1.1 create matrix for sigma points in measurement space
   */
  int n_z = 3; //set measurement dimension, radar can measure r, phi, and r_dot
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  /*
   * 1.2 transform sigma points into measurement space
   */
  VectorXd x;
  VectorXd z_tmp = VectorXd(3);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
      x = Xsig_pred_.col(i);
      z_tmp(0) = sqrt(x(0)*x(0) + x(1)*x(1));
      z_tmp(1) = atan2(x(1), x(0));
      z_tmp(2) = (x(0)*cos(x(3))*x(2) + x(1)*sin(x(3))*x(2)) / z_tmp(0);
      Zsig.col(i) = z_tmp;
	}

  /*
   * 1.3. calculate mean predicted measurement
   */
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
	  z_pred += weights_(i)*Zsig.col(i);
  }

  /*
   * 1.4 calculate innovation covariance matrix S
   */
  MatrixXd S = MatrixXd(n_z, n_z);
  VectorXd z_diff;
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
	  z_diff = Zsig.col(i) - z_pred;
	  //angle normalization
	  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;
	  S += weights_(i)*z_diff*z_diff.transpose();
  }

  /*
   * 1.5 add measurement noise covariance matrix
   */
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0,
	  0, std_radphi_*std_radphi_, 0,
	  0, 0, std_radrd_*std_radrd_;
  S = S + R;

  /*==============================
  *===     2. Update State     ===
  *==============================*/
  /*
   * 2.1 calculate cross correlation matrix
   */
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  VectorXd x_diff;
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
	  x_diff = Xsig_pred_.col(i) - x_;
	  //angle normalization
	  while (x_diff(3)> M_PI) x_diff(3) -= 2.*M_PI;
	  while (x_diff(3)<-M_PI) x_diff(3) += 2.*M_PI;

	  z_diff = Zsig.col(i) - z_pred;
	  //angle normalization
	  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
	  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

	  Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  /*
   * 2.2 calculate Kalman gain K;
   */
  MatrixXd K = Tc * S.inverse();

  /*
   * 2.3 update state mean and covariance matrix
   */
  VectorXd z = VectorXd(3);
  z(0) = meas_package.raw_measurements_(0);
  z(1) = meas_package.raw_measurements_(1);
  z(2) = meas_package.raw_measurements_(2);

  z_diff = z - z_pred;
  while (z_diff(1)> M_PI) z_diff(1) -= 2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1) += 2.*M_PI;

  x_ = x_ + K * z_diff;
  P_ = P_ - K * S*K.transpose();

}
