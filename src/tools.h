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

#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include "render/render.h"
#include <pcl/io/pcd_io.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Lidar marker on another target object
struct lmarker
{
	double x, y;
	lmarker(double setX, double setY)
		: x(setX), y(setY)
	{}
};

// Radar marker on another target object
struct rmarker
{
	double rho, phi, rho_dot;
	rmarker(double setRho, double setPhi, double setRhoDot)
		: rho(setRho), phi(setPhi), rho_dot(setRhoDot)
	{}
};

// Measurement tools class
class Tools
{
	public:

	/**
	 * Constructor.
	 */
	Tools();
	
	/**
	* Destructor.
	*/
	virtual ~Tools();
	
	/* Member variables */

	// Vectors to store UKF estimations and ground truth to calculate overall RMSE values
	std::vector<VectorXd> estimations;
	std::vector<VectorXd> ground_truth;

	/* Member functions */

	// Generate normally distributed random measurement noise	
	double noise(double stddev, long long seedNum);

	// Sense where a car is located relative to the ego car using Lidar measurement
	lmarker lidarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize);
	
	// Sense where a car is located relative to the ego car using Radar measurement
	rmarker radarSense(Car& car, Car ego, pcl::visualization::PCLVisualizer::Ptr& viewer, long long timestamp, bool visualize);
	
	// Show UKF tracking and also allow showing predicted future path
	void ukfResults(Car car, pcl::visualization::PCLVisualizer::Ptr& viewer, double time, int steps);
	
	// Helper method to calculate RMSE
	VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

	// Save point cloud to file
	void savePcd(typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string file);

	// Load point cloud from file
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadPcd(std::string file);
};

#endif /* TOOLS_H_ */
