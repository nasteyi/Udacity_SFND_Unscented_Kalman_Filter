#include <iostream>
#include <random>
#include "tools.h"


double Tools::noise(double stddev, long long seedNum)
{
	std::mt19937::result_type seed = seedNum;

	auto dist = std::bind(
        std::normal_distribution<double>{0, stddev},
        std::mt19937(seed));

	return dist();
}

// sense where a car is located using lidar measurement
lmarker Tools::lidarSense(
    Car& car,
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    long long timestamp,
    bool visualize)
{
	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::LASER;
  	meas_package.raw_measurements_ = Eigen::VectorXd(2);

	const lmarker marker = lmarker(
        car.position.x + noise(0.15, timestamp),
        car.position.y + noise(0.15, timestamp+1));

    if (visualize)
    {
        viewer->addSphere(
            pcl::PointXYZ(marker.x, marker.y, 3.0),
            0.5,
            1,
            0,
            0,
            car.name + "_lmarker");
    }

    meas_package.raw_measurements_ << marker.x, marker.y;
    meas_package.timestamp_ = timestamp;

 
    car.ukf.ProcessMeasurement(meas_package);

    return marker;
}

// sense where a car is located using radar measurement
rmarker Tools::radarSense(
    Car& car,
    const Car& ego,
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    long long timestamp,
    bool visualize)
{
	double rho = std::sqrt(
        (car.position.x - ego.position.x) * (car.position.x - ego.position.x) +
        (car.position.y - ego.position.y) * (car.position.y - ego.position.y));

	double phi = std::atan2(
        car.position.y - ego.position.y,
        car.position.x - ego.position.x);

	double rho_dot = (car.velocity*std::cos(car.angle)*rho*std::cos(phi)
        + car.velocity*std::sin(car.angle)*rho*std::sin(phi))/rho;

	const rmarker marker = rmarker(
        rho + noise(0.3, timestamp + 2),
        phi + noise(0.03, timestamp + 3),
        rho_dot + noise(0.3, timestamp + 4));

	if(visualize)
	{
        const double posX = ego.position.x + marker.rho*std::cos(marker.phi);
        const double posY = ego.position.y + marker.rho*std::sin(marker.phi);

		viewer->addLine(
            pcl::PointXYZ(ego.position.x, ego.position.y, 3.0),
            pcl::PointXYZ(posX, posY, 3.0),
            1,
            0,
            1,
            car.name+"_rho");

		viewer->addArrow(
            pcl::PointXYZ(posX, posY, 3.0),
            pcl::PointXYZ(
                posX + marker.rho_dot*std::cos(marker.phi),
                posY + marker.rho_dot*std::sin(marker.phi),
                3.0),
            1,
            0,
            1,
            car.name+"_rho_dot");
	}

	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = Eigen::VectorXd(3);
    meas_package.raw_measurements_ << marker.rho, marker.phi, marker.rho_dot;
    meas_package.timestamp_ = timestamp;

    car.ukf.ProcessMeasurement(meas_package);

    return marker;
}

// Show UKF tracking and also allow showing predicted future path
// double time:: time ahead in the future to predict
// int steps:: how many steps to show between present and time and future time
void Tools::ukfResults(
    Car& car,
    pcl::visualization::PCLVisualizer::Ptr& viewer,
    double time,
    int steps)
{
	UKF& ukf = car.ukf;

	viewer->addSphere(
        pcl::PointXYZ(ukf.x_[0], ukf.x_[1], 3.5),
        0.5,
        0,
        1,
        0,
        car.name + "_ukf");

	viewer->addArrow(
        pcl::PointXYZ(ukf.x_[0], ukf.x_[1], 3.5),

        pcl::PointXYZ(
            ukf.x_[0] + ukf.x_[2] * std::cos(ukf.x_[3]),
            ukf.x_[1] + ukf.x_[2] * std::sin(ukf.x_[3]),
            3.5),
        0,
        1,
        0,
        car.name+"_ukf_vel");

	if(time > 0)
	{
		double dt = time/steps;
		double ct = dt;
		while(ct <= time)
		{
			ukf.Predict(dt);

			viewer->addSphere(
                pcl::PointXYZ(ukf.x_[0], ukf.x_[1],3.5),
                0.5,
                0,
                1,
                0,
                car.name+"_ukf"+std::to_string(ct));

			viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_OPACITY,
                1.0-0.8*(ct/time),
                car.name+"_ukf"+std::to_string(ct));

            //viewer->addArrow(
            //    pcl::PointXYZ(ukf.x_[0], ukf.x_[1], 3.5),
            //    pcl::PointXYZ(
            //        ukf.x_[0]+ukf.x_[2]*std::cos(ukf.x_[3]),
            //        ukf.x_[1]+ukf.x_[2]*std::sin(ukf.x_[3]),
            //        3.5),
            //    0,
            //    1,
            //    0,
            //    car.name + "_ukf_vel" + std::to_string(ct));

			//viewer->setShapeRenderingProperties(
   //             pcl::visualization::PCL_VISUALIZER_OPACITY,
   //             1.0-0.8*(ct/time),
   //             car.name+"_ukf_vel"+std::to_string(ct));

			ct += dt;
		}
	}

}

Eigen::VectorXd Tools::CalculateRMSE(
    const std::vector<Eigen::VectorXd> &estimations,
    const std::vector<Eigen::VectorXd> &ground_truth)
{
    Eigen::VectorXd rmse = Eigen::VectorXd::Zero(4);

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size() || estimations.size() == 0)
    {
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(std::size_t i = 0; i < estimations.size(); ++i)
    {
		Eigen::VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

void Tools::savePcd(
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    std::string file)
{
      pcl::io::savePCDFileASCII (file, *cloud);
      std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Tools::loadPcd(std::string file)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    //std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

