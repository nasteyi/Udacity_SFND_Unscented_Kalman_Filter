#include "ukf.h"

#include <initializer_list>
#include <iostream>

namespace {

    Eigen::MatrixXd initNoiseMatrix(const std::initializer_list<double>& stds)
    {
        const std::size_t n = stds.size();
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(n, n);
        std::size_t i = 0;
        for (const auto& s : stds)
        {
            R(i, i) = s * s;
            i++;
        }

        return R;
    }

    double normPi(double a)
    {
        a = std::fmod(a, 2 * M_PI);

        if (a >= M_PI)
        {
            a -= 2 * M_PI;
        }
        else if (a < -M_PI)
        {
            a += 2 * M_PI;
        }

        return a;
    }
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
    // if this is false, laser measurements will be ignored (except during init)
    : use_laser_{ true }
    // if this is false, radar measurements will be ignored (except during init)
    , use_radar_{ true }
    // Process noise standard deviation longitudinal acceleration in m/s^2
    , std_a_{ 5.0 }

    // Process noise standard deviation yaw acceleration in rad/s^2
    , std_yawdd_{ 0.8 }

    /**
     * DO NOT MODIFY measurement noise values below.
     * These are provided by the sensor manufacturer.
     */

     // Laser measurement noise standard deviation position1 in m
    , std_laspx_{ 0.1 }

    // Laser measurement noise standard deviation position2 in m
    , std_laspy_{ 0.15 }

    // Radar measurement noise standard deviation radius in m
    , std_radr_{ 0.3 }

    // Radar measurement noise standard deviation angle in rad
    , std_radphi_{ 0.03 }

    // Radar measurement noise standard deviation radius change in m/s
    , std_radrd_{ 0.3 }

    /**
     * End DO NOT MODIFY section for measurement noise values
     */

     /**
      * TODO: Complete the initialization. See ukf.h for other member properties.
      * Hint: one or more values initialized above might be wildly off...
      */
      // Initially set to false, set to true in first call of ProcessMeasurement
    , is_initialized_{ false }

    // State dimension
    , n_x_{ 5 }

    // Augmented state dimension
    , n_aug_{ 7 }

    // Sigma point spreading parameter
    , lambda_{ 3.0 - n_x_ }

    , previous_timestamp_{ 0 }

    // measurement noise covariance matrix
    , R_radar_{ initNoiseMatrix({std_radr_, std_radphi_, std_radrd_}) }
    , R_laser_{ initNoiseMatrix({std_laspx_, std_laspy_}) }
{
    // initial state vector
    x_ = Eigen::VectorXd::Zero(n_x_);

    // initial covariance matrix
    P_ = Eigen::MatrixXd::Zero(n_x_, n_x_);

    // predicted sigma points matrix
    Xsig_pred_ = Eigen::MatrixXd::Zero(n_x_, 2 * n_aug_ + 1);

    // Weights of sigma points
    weights_ = Eigen::VectorXd::Zero(2 * n_aug_ + 1);
}

void UKF::ProcessMeasurement(const MeasurementPackage& meas_package)
{
    /**
     * TODO: Complete this function! Make sure you switch between LiDAR and radar
     * measurements.
     */
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_)
    {
        return;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_)
    {
        return;
    }

    double dt = 1e-6 * static_cast<double>(
        meas_package.timestamp_ - previous_timestamp_);

    previous_timestamp_ = meas_package.timestamp_;   

    // Initialization
    if (!is_initialized_)
    {
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            const double ro = meas_package.raw_measurements_(0);
            const double phi = meas_package.raw_measurements_(1);
            x_ << ro * std::cos(phi), ro * std::sin(phi), 0, 0, 0;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
        }
        else
        {
            assert(0 && "unknown sensor");
        }

        is_initialized_ = true;
        return;
    }

    /*
     * Prediction
     * Update the state transition matrix F according to the new elapsed time.
     * Update the process noise covariance matrix */

    // Improve the stability by subdividing the prediction step for large delta_ts into incremental updates
    // Otherwise Cholesky decomposition may fail
    const double delta_t = 0.05;

    while (dt > 2 * delta_t)
    {
        Predict(delta_t);
        dt -= delta_t;
    }

    Predict(dt);

    /*
     * Update
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
    */
    n_z_ = 0;
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        //set measurement dimension, radar can measure r, phi, and r_dot
        n_z_ = 3;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        //set measurement dimension, laser can measure px, py
        n_z_ = 2;
    }
    else
    {
        assert(0 && "unknown sensor");
    }

    //create matrix for sigma points in measurement space
    Zsig_ = Eigen::MatrixXd::Zero(n_z_, 2 * n_aug_ + 1);

    //mean predicted measurement
    z_pred_ = Eigen::VectorXd::Zero(n_z_);

    //measurement covariance matrix S
    S_ = Eigen::MatrixXd::Zero(n_z_, n_z_);

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
        PredictRadarMeasurement();
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
        PredictLaserMeasurement();
    }
    else
    {
        assert(0 && "unknown sensor");
    }

    UpdateState(meas_package.raw_measurements_);

}


void UKF::Predict(double dt)
{
    /**
     * TODO: Complete this function! Estimate the object's location.
     * Modify the state vector, x_. Predict sigma points, the state,
     * and the state covariance matrix.
     */
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
    GenerateAugmentedSigmaPoints(Xsig_aug);
    SigmaPointPrediction(Xsig_aug, dt);
    PredictMeanAndCovariance();

}


void UKF::GenerateAugmentedSigmaPoints(Eigen::MatrixXd& Xsig_aug)
{
    //create augmented mean vector
    Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug_);

    //create augmented state covariance
    Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_);

    //create augmented mean state
    x_aug.head(n_x_) = x_;

    //create augmented covariance matrix
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    // Cholesky decomposition
    Eigen::MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0) = x_aug;
    const double k = std::sqrt(lambda_ + n_aug_);
    for (int i = 0; i < n_aug_; ++i)
    {
        Xsig_aug.col(i + 1) = x_aug + k * L.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug - k * L.col(i);
    }
}


void UKF::SigmaPointPrediction(const Eigen::MatrixXd& Xsig_aug, double dt)
{
    //predict sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_p = 0;
        double py_p = 0;

        double cosYaw = std::cos(yaw);
        double sinYaw = std::sin(yaw);

        //avoid division by zero
        if (std::abs(yawd) > std::numeric_limits<double>::epsilon())
        {
            px_p = p_x + v / yawd * (std::sin(yaw + yawd * dt) - sinYaw);
            py_p = p_y + v / yawd * (cosYaw - std::cos(yaw + yawd * dt));
        }
        else
        {
            px_p = p_x + v * dt * cosYaw;
            py_p = p_y + v * dt * sinYaw;
        }

        double v_p = v;
        double yaw_p = yaw + yawd * dt;
        double yawd_p = yawd;

        double dt2 = dt * dt;
        //add noise
        px_p += 0.5 * nu_a * dt2 * cosYaw;
        py_p += 0.5 * nu_a * dt2 * sinYaw;
        v_p += nu_a * dt;

        yaw_p += + 0.5 * nu_yawdd * dt2;
        yawd_p += nu_yawdd * dt;

        //write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
}


void UKF::PredictMeanAndCovariance()
{
    const double den = 1.0 / (lambda_ + n_aug_);

    // set weights and these weights will be shared during update
    weights_(0) = lambda_ * den;
    for (int i = 1; i < 2 * n_aug_ + 1; ++i)
    {
        weights_(i) = 0.5 * den;
    }

    //predicted state mean
    x_.fill(0.0);
    //iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        x_ += weights_(i) * Xsig_pred_.col(i);
    }

    //predicted state covariance matrix
    P_.fill(0.0);
    //iterate over sigma points
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // state difference
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //angle normalization
        x_diff(3) = normPi(x_diff(3));

        P_ += weights_(i) * x_diff * x_diff.transpose();
    }
}


void UKF::PredictRadarMeasurement()
{
   
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {  //2n+1 sigma points

        // extract values for better readability
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);

        double v1 = std::cos(yaw)*v;
        double v2 = std::sin(yaw)*v;

        // measurement model
        Zsig_(0, i) = std::sqrt(p_x*p_x + p_y * p_y);                   //r
        Zsig_(1, i) = std::atan2(p_y, p_x);                            //phi

        if (Zsig_(0, i) < 0.001)
        {
            Zsig_(2, i) = (p_x * v1 + p_y * v2) / 0.001;        //r_dot
        }
        else
        {
            Zsig_(2, i) = (p_x * v1 + p_y * v2) / Zsig_(0, i);  //r_dot;
        }
    }

    z_pred_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
    }

    S_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        //residual
        Eigen::VectorXd z_diff = Zsig_.col(i) - z_pred_;

        //angle normalization
        z_diff(1) = normPi(z_diff(1));

        S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
    }

    S_ = S_ + R_radar_;
}


void UKF::PredictLaserMeasurement()
{
    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        // measurement model
        Zsig_(0, i) = Xsig_pred_(0, i);           //px
        Zsig_(1, i) = Xsig_pred_(1, i);           //py
    }

    z_pred_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
    }

    S_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {
        //residual
        Eigen::VectorXd z_diff = Zsig_.col(i) - z_pred_;

        // angle normalization
        z_diff(1) = normPi(z_diff(1));

        S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
    }

    S_ = S_ + R_laser_;
}


void UKF::UpdateState(const Eigen::VectorXd& z)
{
    //create matrix for cross correlation Tc
    Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z_);

    //calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug_ + 1; ++i)
    {  //2n+1 sigma points

        //residual
        Eigen::VectorXd z_diff = Zsig_.col(i) - z_pred_;

        //angle normalization
        z_diff(1) = normPi(z_diff(1));

        // state difference
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = normPi(x_diff(3));

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    //Kalman gain K;
    const Eigen::MatrixXd K = Tc * S_.inverse();

    //residual
    Eigen::VectorXd z_diff = z - z_pred_;

    //angle normalization
    z_diff(1) = normPi(z_diff(1));

    //update state mean and covariance matrix
    x_ = x_ + K * z_diff;
    P_ = P_ - K * S_ * K.transpose();
}