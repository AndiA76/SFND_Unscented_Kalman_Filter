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

// Tools to simulate Lidar and Radar measurements on other target objects

#include <iostream>
#include <random>
#include "tools.h"

using namespace std;
using std::vector;

// Constructor of Tools() class
Tools::Tools() {}

// Destructor of Tools() class
Tools::~Tools() {}

// Generate normally distributed random measurement noise
double Tools::noise(double stddev, long long seedNum)
{
	mt19937::result_type seed = seedNum;
	auto dist = std::bind(std::normal_distribution<double>{0, stddev}, std::mt19937(seed));
	return dist();
}

// Sense where a car is located relative to the ego car using Lidar measurement (calculated in the inertial coordinate system!)
lmarker Tools::lidarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize)
{
	// Initialize data structure for next Lidar measurement
	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::LASER;
  	meas_package.raw_measurements_ = VectorXd(2);

	// Set Lidar marker for the current target object detection are relative to the ego car
	lmarker marker = lmarker(
		car.position.x - ego.position.x + noise(0.15, timestamp),
		car.position.y - ego.position.y + noise(0.15, timestamp + 1));
	if(visualize)
		viewer->addSphere(pcl::PointXYZ(marker.x,marker.y,3.0),0.5, 1, 0, 0,car.name+"_lmarker");
	
	// Store next Lidar measurement (given in cartesian coordinates relative to the ego car)
    meas_package.raw_measurements_ << marker.x, marker.y;
    meas_package.timestamp_ = timestamp;

	// Trigger Kalman Filter process measurement routine for the current car
    car.ukf.ProcessMeasurement(meas_package);

	// Return Lidar marker
    return marker;
}

// Sense where a car is located relative to the ego car using Radar measurement (calculated in the inertial coordinate system!)
rmarker Tools::radarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize)
{
	// Initialize data structure for next Radar measurement
	MeasurementPackage meas_package;
	meas_package.sensor_type_ = MeasurementPackage::RADAR;
    meas_package.raw_measurements_ = VectorXd(3);

	// Calculate distance to the target object in radial direction
	double rho = sqrt((car.position.x-ego.position.x)*(car.position.x-ego.position.x)+(car.position.y-ego.position.y)*(car.position.y-ego.position.y));
	
	// Calculate bearing angle to the target object with respect to the ego longitudinal axis assuming the ego vehicle always goes straight
	double phi = atan2(car.position.y-ego.position.y,car.position.x-ego.position.x);

	// This original formula assumes a constant ego velocity and ego driving direction for the relative velocity measurement
	double rho_dot = (car.velocity*cos(car.angle)*rho*cos(phi) + car.velocity*sin(car.angle)*rho*sin(phi))/rho;

	/* Alternative calculation considering changing ego velocity and driving direction as well in an inertial coordinate system
	
	// Alternative calculation of the bearing angle to the target boject with respect to the ego longitudinal axis, which might change in this case
	double phi = atan2(car.position.y-ego.position.y,car.position.x-ego.position.x) - ego.angle;
		
	// Calculate absolute magnitude of the relative velocity
	double rel_velocity = sqrt(
		(car.velocity*cos(car.angle) - ego.velocity*cos(ego.angle)) * (car.velocity*cos(car.angle) - ego.velocity*cos(ego.angle)) +
		(car.velocity*sin(car.angle) - ego.velocity*sin(ego.angle)) * (car.velocity*sin(car.angle) - ego.velocity*sin(ego.angle)));
	
	// Calculate orientation angle of the relative velocity vector with respect to the global x-y-coordinate system
	double rel_velocity_angle = atan2((car.velocity*sin(car.angle) - ego.velocity*sin(ego.angle)), (car.velocity*cos(car.angle) - ego.velocity*cos(ego.angle)));

	// Calculate relative Doppler velocity in radial direction, resp. the radial relative velocity component (The tangential relative velocity component is not measurable)
	double rho_dot = rel_velocity * cos(ego.angle + phi - rel_velocity_angle);

	*/

	// Set Radar marker for the current target object detection relative to the ego car
	rmarker marker = rmarker(rho+noise(0.3,timestamp+2), phi+noise(0.03,timestamp+3), rho_dot+noise(0.3,timestamp+4));
	if(visualize)
	{
		viewer->addLine(pcl::PointXYZ(ego.position.x, ego.position.y, 3.0), pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi), 3.0), 1, 0, 1, car.name+"_rho");
		viewer->addArrow(pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi), 3.0), pcl::PointXYZ(ego.position.x+marker.rho*cos(marker.phi)+marker.rho_dot*cos(marker.phi), ego.position.y+marker.rho*sin(marker.phi)+marker.rho_dot*sin(marker.phi), 3.0), 1, 0, 1, car.name+"_rho_dot");
	}
	
	// Store next Radar measurement (given in polar coordinates)
    meas_package.raw_measurements_ << marker.rho, marker.phi, marker.rho_dot;
    meas_package.timestamp_ = timestamp;

	// Trigger Kalman Filter process measurement routine for the current car
    car.ukf.ProcessMeasurement(meas_package);

	// Return Radar marker
    return marker;
}

// Show UKF tracking and also allow showing predicted future path
// double time:: time ahead in the future to predict
// int steps:: how many steps to show between present and time and future time
void Tools::ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps)
{
	UKF ukf = car.ukf;
	viewer->addSphere(pcl::PointXYZ(ukf.x_[0],ukf.x_[1],3.5), 0.5, 0, 1, 0,car.name+"_ukf");
	viewer->addArrow(pcl::PointXYZ(ukf.x_[0], ukf.x_[1],3.5), pcl::PointXYZ(ukf.x_[0]+ukf.x_[2]*cos(ukf.x_[3]),ukf.x_[1]+ukf.x_[2]*sin(ukf.x_[3]),3.5), 0, 1, 0, car.name+"_ukf_vel");
	if(time > 0)
	{
		double dt = time/steps;
		double ct = dt;
		while(ct <= time)
		{
			ukf.Prediction(dt);
			viewer->addSphere(pcl::PointXYZ(ukf.x_[0],ukf.x_[1],3.5), 0.5, 0, 1, 0,car.name+"_ukf"+std::to_string(ct));
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-0.8*(ct/time), car.name+"_ukf"+std::to_string(ct));
			//viewer->addArrow(pcl::PointXYZ(ukf.x_[0], ukf.x_[1],3.5), pcl::PointXYZ(ukf.x_[0]+ukf.x_[2]*cos(ukf.x_[3]),ukf.x_[1]+ukf.x_[2]*sin(ukf.x_[3]),3.5), 0, 1, 0, car.name+"_ukf_vel"+std::to_string(ct));
			//viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0-0.8*(ct/time), car.name+"_ukf_vel"+std::to_string(ct));
			ct += dt;
		}
	}

}

// Calculate Root-Mean-Square-Error
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth)
{
	// Initialize RMSE values
    VectorXd rmse(4);
	rmse << 0,0,0,0;

	// Check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	// Accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		// Coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// Calculate the mean
	rmse = rmse/estimations.size();

	// Calculate the squared root
	rmse = rmse.array().sqrt();

	// Return the result
	return rmse;
}

// Save point cloud to file
void Tools::savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file)
{
  pcl::io::savePCDFileASCII (file, *cloud);
  std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}

// Load point cloud from file
pcl::PointCloud<pcl::PointXYZ>::Ptr Tools::loadPcd(std::string file)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
  }
  //std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

  return cloud;
}