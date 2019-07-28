#pragma once

#include "Eigen/Dense"
#include "measurement_package.h"

class UKF {
public:
    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF() = default;

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param dt Time between k and k+1 in s
     */
    void Predict(double dt);

    void GenerateAugmentedSigmaPoints(Eigen::MatrixXd& Xsig_out);
    void SigmaPointPrediction(const Eigen::MatrixXd& Xsig_aug, double dt);
    void PredictMeanAndCovariance();
    void PredictRadarMeasurement();
    void PredictLaserMeasurement();
    void UpdateState(const Eigen::VectorXd& z);


    // initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_{ false };

    // if this is false, laser measurements will be ignored (except for init)
    const bool use_laser_;

    // if this is false, radar measurements will be ignored (except for init)
    const bool use_radar_;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // predicted sigma points matrix
    Eigen::MatrixXd Xsig_pred_;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    const double std_a_;

    // Process noise standard deviation yaw acceleration in rad/s^2
    const double std_yawdd_;

    // Laser measurement noise standard deviation position1 in m
    const double std_laspx_;

    // Laser measurement noise standard deviation position2 in m
    const double std_laspy_;

    // Radar measurement noise standard deviation radius in m
    const double std_radr_;

    // Radar measurement noise standard deviation angle in rad
    const double std_radphi_;

    // Radar measurement noise standard deviation radius change in m/s
    const double std_radrd_;

    // Weights of sigma points
    Eigen::VectorXd weights_;

    // State dimension
    const int n_x_;

    // Augmented state dimension
    const int n_aug_;

    // Sigma point spreading parameter
    const double lambda_;

    // current NIS for radar
    double NIS_radar_;

    // current NIS for laser
    double NIS_laser_;

private:
    // previous timestamp
    long previous_timestamp_{ -1 };

    //set measurement dimension, radar can measure r, phi, and r_dot
    int n_z_{ 0 };

    //create matrix for sigma points in measurement space
    Eigen::MatrixXd Zsig_;

    //mean predicted measurement
    Eigen::VectorXd z_pred_;

    //measurement covariance matrix S
    Eigen::MatrixXd S_;

    //measurement noise covariance matrix
    const Eigen::MatrixXd R_laser_;
    const Eigen::MatrixXd R_radar_;

};