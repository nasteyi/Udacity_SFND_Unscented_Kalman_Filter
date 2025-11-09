#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

struct MeasurementPackage {
    long long timestamp_{-1};

    enum SensorType { LASER, RADAR } sensor_type_;

    Eigen::VectorXd raw_measurements_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */