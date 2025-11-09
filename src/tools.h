#ifndef TOOLS_H_
#define TOOLS_H_
#include <pcl/io/pcd_io.h>

#include <vector>

#include "Eigen/Dense"
#include "render/render.h"

struct lmarker {
    double x{0};
    double y{0};

    explicit lmarker(double setX, double setY) : x(setX), y(setY) {}
};

struct rmarker {
    double rho{0};
    double phi{0};
    double rho_dot{0};

    explicit rmarker(double setRho, double setPhi, double setRhoDot) : rho(setRho), phi(setPhi), rho_dot(setRhoDot) {}
};

class Tools {
public:
    /**
     * Constructor.
     */
    Tools() = default;

    /**
     * Destructor.
     */
    virtual ~Tools() = default;

    // Members
    std::vector<Eigen::VectorXd> estimations;
    std::vector<Eigen::VectorXd> ground_truth;

    static double noise(double stddev, long long seedNum);

    static lmarker lidarSense(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp,
                              bool visualize);

    static rmarker radarSense(Car& car, const Car& ego, pcl::visualization::PCLVisualizer::Ptr& viewer,
                              long long timestamp, bool visualize);

    static void ukfResults(Car& car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps);

    /**
     * A helper method to calculate RMSE.
     */
    static Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd>& estimations,
                                         const std::vector<Eigen::VectorXd>& ground_truth);

    static void savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file);
};

#endif /* TOOLS_H_ */
