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

// Handle logic for creating and animating a 3D highway scenario with traffic

#include "render/render.h"
#include "sensors/lidar.h"
#include "tools.h"

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision
#include <fstream>		// std::ofstream

//#include "gnuplot_i.hpp" //Gnuplot class handles POSIX-Pipe-communikation with Gnuplot
//#include "gnuplot-iostream.h"

// Delare highway simulator class
class Highway
{
public:

	// Initialize data structures for simulation
	// -----------------------------------------------------------
	std::vector<Car> traffic;
	Car egoCar;
	Tools tools;
	bool pass = true;
	std::vector<double> rmseThreshold = {0.30,0.16,0.95,0.70};
	std::vector<double> rmseFailLog = {0.0,0.0,0.0,0.0};
	Lidar* lidar;
	// -----------------------------------------------------------
	
	// Parameters 
	// -----------------------------------------------------------
	// Set which cars to track with UKF
	std::vector<bool> trackCars = {true,true,true};
	// Visualize sensor measurements
	bool visualize_lidar = true;
	bool visualize_radar = true;
	bool visualize_pcd = false;
	// Predict path in the future using UKF
	double projectedTime = 1; // (0 | 1): path prediction (off | on)
	int projectedSteps = 5; // number of projected time steps (default: 0)
	// -----------------------------------------------------------

	// Initialize file buffer for data logging
	// -----------------------------------------------------------
	const char* data_log_filepath = "../ukf_output/data_log.csv";
	std::ofstream data_log;
	// -----------------------------------------------------------

	// Constructor: Initialize highway scenario with traffic
	Highway(pcl::visualization::PCLVisualizer::Ptr& viewer)
	{
		// Set up Lidar and Radar measurement tools
		tools = Tools();

		// Create an ego car always going straight with constant velocity without changing relative position
		egoCar = Car(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), 0, 0, 2, "egoCar");
		
		// Create target care 1
		Car car1(Vect3(-10, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), 5, 0, 2, "car1");
		
		// Define driving instructions for target car 1
		std::vector<accuation> car1_instructions;
		accuation a = accuation(0.5*1e6, 0.5, 0.0);
		car1_instructions.push_back(a);
		a = accuation(2.2*1e6, 0.0, -0.2);
		car1_instructions.push_back(a);
		a = accuation(3.3*1e6, 0.0, 0.2);
		car1_instructions.push_back(a);
		a = accuation(4.4*1e6, -2.0, 0.0);
		car1_instructions.push_back(a);	
		car1.setInstructions(car1_instructions);

		// Set up an Unscented Kalman Filter for target car 1
		if( trackCars[0] )
		{
			UKF ukf1;
			car1.setUKF(ukf1);
		}
		traffic.push_back(car1);
		
		// Create target car 2
		Car car2(Vect3(25, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), -6, 0, 2, "car2");

		// Define driving instructions for target car 2
		std::vector<accuation> car2_instructions;
		a = accuation(4.0*1e6, 3.0, 0.0);
		car2_instructions.push_back(a);
		a = accuation(8.0*1e6, 0.0, 0.0);
		car2_instructions.push_back(a);
		car2.setInstructions(car2_instructions);

		// Set up an Unscented Kalman Filter for target car 2
		if( trackCars[1] )
		{
			UKF ukf2;
			car2.setUKF(ukf2);
		}
		traffic.push_back(car2);
	
		// Create target car 3
		Car car3(Vect3(-12, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), 1, 0, 2, "car3");

		// Define driving instructions for target car 3
		std::vector<accuation> car3_instructions;
		a = accuation(0.5*1e6, 2.0, 1.0);
		car3_instructions.push_back(a);
		a = accuation(1.0*1e6, 2.5, 0.0);
		car3_instructions.push_back(a);
		a = accuation(3.2*1e6, 0.0, -1.0);
		car3_instructions.push_back(a);
		a = accuation(3.3*1e6, 2.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(4.5*1e6, 0.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(5.5*1e6, -2.0, 0.0);
		car3_instructions.push_back(a);
		a = accuation(7.5*1e6, 0.0, 0.0);
		car3_instructions.push_back(a);
		car3.setInstructions(car3_instructions);

		// Set up an Unscented Kalman Filter for target car 3
		if( trackCars[2] )
		{
			UKF ukf3;
			car3.setUKF(ukf3);
		}
		traffic.push_back(car3);

		// Set up a structure for 3D Lidar point cloud objects
		lidar = new Lidar(traffic,0);
	
		// Render environment
		renderHighway(0,viewer);
		egoCar.render(viewer);
		car1.render(viewer);
		car2.render(viewer);
		car3.render(viewer);
	}
	
	// Simulate the next time step of the highway scenario
	void stepHighway(double egoVelocity, long long timestamp, int frame_per_sec, pcl::visualization::PCLVisualizer::Ptr& viewer)
	{
		// Visualize Lidar point cloud
		if(visualize_pcd)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr trafficCloud = tools.loadPcd("../src/sensors/data/pcd/highway_"+std::to_string(timestamp)+".pcd");
			renderPointCloud(viewer, trafficCloud, "trafficCloud", Color((float)184/256,(float)223/256,(float)252/256));
		}
		
		// Render highway environment with poles at both sides of the highway
		renderHighway(egoVelocity*timestamp/1e6, viewer);
		egoCar.render(viewer);

		// Simulate the next time step for each car on the highway
		for (int i = 0; i < traffic.size(); i++)
		{
			traffic[i].move((double)1/frame_per_sec, timestamp);
			if(!visualize_pcd)
				traffic[i].render(viewer);
			// Sense surrounding cars with lidar and radar
			if(trackCars[i])
			{
				/* Get ground truth states of the current target car relative to the ego car */
				// Get x-position of the target car relative to ego car
				double px_gt = traffic[i].position.x - egoCar.position.x;
				// Get y-position of the target car relative to ego car
				double py_gt = traffic[i].position.y - egoCar.position.y;
				// Get instantaneous velocity of the target car along its longitudinal axis
				double v_gt = traffic[i].velocity;
				// Get yaw angle of the target car (ego vehicle is going straight)
				double yaw_gt = traffic[i].angle;
				// Get instantaneous yaw rate of the target car (ego vehicle is going straight)
				double yaw_rate_gt = traffic[i].yaw_rate;
				// Get velocity component of the target car in x-direction relative to the ego car
				double vx_gt = (
					traffic[i].velocity * cos(traffic[i].angle)
					- egoCar.velocity * cos(egoCar.angle));
				// Get velocity component of the target car in y-direction relative to the ego car
				double vy_gt = (
					traffic[i].velocity * sin(traffic[i].angle)
					- egoCar.velocity * sin(egoCar.angle));
				// Get acceleration component in x-direction relative to the ego car
				double ax_gt = (
					traffic[i].acceleration * cos(traffic[i].angle)
					- egoCar.acceleration * cos(egoCar.angle));
				// Get acceleration component in y-direction relative to the ego car
				double ay_gt = (
					traffic[i].acceleration * sin(traffic[i].angle)
					- egoCar.acceleration * sin(egoCar.angle));
				// Get instantaneous acceleration of the target car along its longitudinal axis
				double a_gt = traffic[i].acceleration;

				// Set ground truth states of the current target vehicle for RMSE calculation
				VectorXd gt(4);
				gt << px_gt, py_gt, vx_gt, vy_gt;
				tools.ground_truth.push_back(gt);

				// Take a Lidar measurement of the current target vehicle relative to the ego car
				tools.lidarSense(traffic[i], egoCar, viewer, timestamp, visualize_lidar);

				// Take a Radar measurement of the current target vehicle relative to the ego car
				tools.radarSense(traffic[i], egoCar, viewer, timestamp, visualize_radar);

				// Show tracking and predicted future path (if activated)
				tools.ukfResults(traffic[i],viewer, projectedTime, projectedSteps);

				/* Get estimated states from ukf for the current target vehicle relative to the ego car */
				// Get x-position relative to ego car
				double px_ukf = traffic[i].ukf.x_[0];
				// Get y-position relative to ego car
				double py_ukf = traffic[i].ukf.x_[1];
				// Get instantaneous velocity relative to the ego car
				double v_ukf = traffic[i].ukf.x_[2];
				// Get yaw angle relative to the ego car
				double yaw_ukf = traffic[i].ukf.x_[3];
				// Get instantaneous yaw rate relative to the ego car
				double yaw_rate_ukf = traffic[i].ukf.x_[4];
				// Get velocity component in x-direction relative to the ego car
    			double vx_ukf = v_ukf * cos(yaw_ukf);
				// Get velocity component in y-direction relative to the ego car
    			double vy_ukf = v_ukf * sin(yaw_ukf);

				// Set the state estimate from ukf associated with the current target vehicle for RMSE calculation
				VectorXd estimate(4);
				estimate << px_ukf, py_ukf, vx_ukf, vy_ukf;
				tools.estimations.push_back(estimate);

				// Log timestamp
				data_log<<std::fixed<<std::setprecision(0)<<traffic[i].ukf.time_us_<<",";

				// Log ground truth states (plus acceleration) relative to the ego vehicle
				data_log<<std::fixed<<std::setprecision(9)<<px_gt<<","; // x-position relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<py_gt<<","; // y-position relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<v_gt<<","; // instantaneous velocity in longitudinal direction
				data_log<<std::fixed<<std::setprecision(9)<<yaw_gt<<","; // yaw angle
				data_log<<std::fixed<<std::setprecision(9)<<yaw_rate_gt<<","; // instantaneous yaw rate
				// additional
				data_log<<std::fixed<<std::setprecision(9)<<vx_gt<<","; // velocity component in x-direction relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<vy_gt<<","; // velocity component in y-direction relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<a_gt<<","; // instantaneous acceleration in longitudinal direction

				// Log estimated states relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<px_ukf<<",";  // x-position relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<py_ukf<<",";  // y-position relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<v_ukf<<",";  // instantaneous velocity relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<yaw_ukf<<",";  // yaw angle relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<yaw_rate_ukf<<",";  // instantaneous yaw rate relative to ego car
				// additional
				data_log<<std::fixed<<std::setprecision(9)<<vx_ukf<<","; // velocity component in x-direction relative to ego car
				data_log<<std::fixed<<std::setprecision(9)<<vy_ukf<<","; // velocity component in y-direction relative to ego car

				// Log state estimation errors = gt - ukf_prediction
				data_log<<std::fixed<<std::setprecision(9)<<px_gt-px_ukf<<","; // x-position estimation error
				data_log<<std::fixed<<std::setprecision(9)<<py_gt-py_ukf<<","; // y-position estimation error
				data_log<<std::fixed<<std::setprecision(9)<<v_gt-v_ukf<<","; // velocity estimation error
				data_log<<std::fixed<<std::setprecision(9)<<yaw_gt-yaw_ukf<<","; // yaw angle estimation error
				data_log<<std::fixed<<std::setprecision(9)<<yaw_rate_gt-yaw_rate_ukf<<","; // yaw rate estimation error
				// additional
				data_log<<std::fixed<<std::setprecision(9)<<vx_gt-vx_ukf<<","; // velocity estimation error in x-direction
				data_log<<std::fixed<<std::setprecision(9)<<vy_gt-vy_ukf<<","; // velocity estimation error in y-direction
				
				// Log normalized innovation squared (NIS) for Lidar and Radar
				data_log<<std::fixed<<std::setprecision(9)<<traffic[i].ukf.NIS_lidar_<<",";
				data_log<<std::fixed<<std::setprecision(9)<<traffic[i].ukf.NIS_radar_<<",";
			}
		}
		// Finish current data log row
		data_log<<"\n";

		// Add accuracy measured by RMSE values for each state to the animated plot
		viewer->addText("Accuracy - RMSE:", 30, 300, 20, 1, 1, 1, "rmse");
		VectorXd rmse = tools.CalculateRMSE(tools.estimations, tools.ground_truth);
		viewer->addText(" X: "+std::to_string(rmse[0]), 30, 275, 20, 1, 1, 1, "rmse_x");
		viewer->addText(" Y: "+std::to_string(rmse[1]), 30, 250, 20, 1, 1, 1, "rmse_y");
		viewer->addText("Vx: "	+std::to_string(rmse[2]), 30, 225, 20, 1, 1, 1, "rmse_vx");
		viewer->addText("Vy: "	+std::to_string(rmse[3]), 30, 200, 20, 1, 1, 1, "rmse_vy");

		// Print RMSE values for each state to the standard output
		std::cout<<"Accuracy - RMSE:"<<std::endl;
		std::cout<<" X: "<<rmse[0]<<" (threshold = "<<rmseThreshold[0]<<")"<<std::endl;
		std::cout<<" Y: "<<rmse[1]<<" (threshold = "<<rmseThreshold[1]<<")"<<std::endl;
		std::cout<<"Vx: "<<rmse[2]<<" (threshold = "<<rmseThreshold[2]<<")"<<std::endl;
		std::cout<<"Vy: "<<rmse[3]<<" (threshold = "<<rmseThreshold[3]<<")"<<std::endl;

		// Check for RMSE thresholds (failed/passed)
		if(timestamp > 1.0e6)
		{
			if(rmse[0] > rmseThreshold[0])
			{
				rmseFailLog[0] = rmse[0];
				pass = false;
			}
			if(rmse[1] > rmseThreshold[1])
			{
				rmseFailLog[1] = rmse[1];
				pass = false;
			}
			if(rmse[2] > rmseThreshold[2])
			{
				rmseFailLog[2] = rmse[2];
				pass = false;
			}
			if(rmse[3] > rmseThreshold[3])
			{
				rmseFailLog[3] = rmse[3];
				pass = false;
			}
		}
		if(!pass)
		{
			// Add failure message with failed RMSE thresholds to the animated plot
			viewer->addText("RMSE Failed Threshold", 30, 150, 20, 1, 0, 0, "rmse_fail");
			if(rmseFailLog[0] > 0)
				viewer->addText(" X: "+std::to_string(rmseFailLog[0]), 30, 125, 20, 1, 0, 0, "rmse_fail_x");
			if(rmseFailLog[1] > 0)
				viewer->addText(" Y: "+std::to_string(rmseFailLog[1]), 30, 100, 20, 1, 0, 0, "rmse_fail_y");
			if(rmseFailLog[2] > 0)
				viewer->addText("Vx: "+std::to_string(rmseFailLog[2]), 30, 75, 20, 1, 0, 0, "rmse_fail_vx");
			if(rmseFailLog[3] > 0)
				viewer->addText("Vy: "+std::to_string(rmseFailLog[3]), 30, 50, 20, 1, 0, 0, "rmse_fail_vy");
		}		
	}

	// Initialize data log
	void initDataLog()
	{
		// Open data log file
		data_log.open(data_log_filepath);

		// Initialize header for data logging
		for (int i = 0; i < traffic.size(); i++)
		{
			// Log timestamp
			data_log<<"time [µs] ("<<traffic[i].name<<"),";
			
			// Log ground truth states relative to ego car
			data_log<<"gt rel. x-position [m] ("<<traffic[i].name<<"),";
			data_log<<"gt rel. y-position [m] ("<<traffic[i].name<<"),";
			data_log<<"gt velocity [m/s] ("<<traffic[i].name<<"),";
			data_log<<"gt yaw angle [rad] ("<<traffic[i].name<<"),";
			data_log<<"gt yaw rate [rad/s] ("<<traffic[i].name<<"),";
			// additional
			data_log<<"gt rel. x-velocity [m/s] ("<<traffic[i].name<<"),";
			data_log<<"gt rel. y-velocity [m/s] ("<<traffic[i].name<<"),";
			data_log<<"gt acceleration [m/s^2] ("<<traffic[i].name<<"),";

			// Log estimated states relative to ego car
			data_log<<"ukf rel. x-position [m] ("<<traffic[i].name<<"),";
			data_log<<"ukf rel. y-position [m] ("<<traffic[i].name<<"),";
			data_log<<"ukf velocity [m/s] ("<<traffic[i].name<<"),";
			data_log<<"ukf yaw angle [rad] ("<<traffic[i].name<<"),";
			data_log<<"ukf yaw rate [rad/s] ("<<traffic[i].name<<"),";
			// additional
			data_log<<"ukf rel. x-velocity [m/s] ("<<traffic[i].name<<"),";
			data_log<<"ukf rel. y-velocity [m/s] ("<<traffic[i].name<<"),";

			// Log state estimation errors
			data_log<<"x-position error [m] ("<<traffic[i].name<<"),";
			data_log<<"y-position error [m] ("<<traffic[i].name<<"),";
			data_log<<"velocity error [m/s] ("<<traffic[i].name<<"),";
			data_log<<"yaw angle error [rad] ("<<traffic[i].name<<"),";
			data_log<<"yaw rate error [rad/s] ("<<traffic[i].name<<"),";
			// additional
			data_log<<"x-velocity error [m/s] ("<<traffic[i].name<<"),";
			data_log<<"y-velocity error [m/s] ("<<traffic[i].name<<"),";

			// Log normalized innovation squared (NIS) for Lidar and Radar
			data_log<<"NIS Lidar ("<<traffic[i].name<<"),";
			data_log<<"NIS Radar ("<<traffic[i].name<<"),";
		}
		// Finish current data log row
		data_log<<"\n";
	}

	// Close data log
	void closeDataLog()
	{
		// Close data log file
		data_log.close();
	}

	// Plot log data
	void plotLogData()
	{
		// Plot logged states and state estimation errors using GNUPLOT

		// Open a gnuplot pipe (gp is the pipe descriptor)
		FILE *gp = popen("gnuplot -persist", "w");
		if (gp==NULL)
		{
			std::cout << "Error opening pipe to GNUPLOT. Check if you have it installed!" << std::endl;
			exit(0);
		}
		// Specify data file separator
		fprintf(gp, "set datafile separator ','\n");

		// Initialize plot counter and the number of columns per car
		int plot_no = 0;
		int num_of_cols_per_car = 25;

		// Loop over all cars simulated in the highway scenario
		for (int i = 0; i < traffic.size(); i++)
		{
			// Get current car name
			const char* car_name = traffic[i].name.c_str();

			/* --- Plot ground truth x-position and estimated x-position with state estimation error --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			// Set multiplot mode: plot multiple plots in one graph
			fprintf(gp, "set multiplot\n");
			// Adjust graph size for 2 x 1 subplots
			fprintf(gp, "set size 1, 0.5\n");

			fprintf(gp, "set origin 0.0, 0.5\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'x-position (%s) relative to ego car'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'x-position in [m]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader, '' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 2,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 10);
		
			fprintf(gp, "set origin 0.0, 0.0\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'x-position state estimation error (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'estimation error in [m]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 17);
			
			// Unset multiplot mode to be able to change the window
			fprintf(gp, "unset multiplot\n");
			// Unset plot size 
			fprintf(gp, "unset size\n");
			
			/* --- Plot ground truth y-position and estimated state y-position with state estimation error --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			// Set multiplot mode: plot multiple plots in one graph
			fprintf(gp, "set multiplot\n");
			// Adjust graph size for 2 x 1 subplots
			fprintf(gp, "set size 1, 0.5\n");

			fprintf(gp, "set origin 0.0, 0.5\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'y-position (%s) relative to ego car'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'y-position in [m]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader, '' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 3,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 11);
		
			fprintf(gp, "set origin 0.0, 0.0\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'y-position state estimation error (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'estimation error in [m]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 18);
			
			// Unset multiplot mode to be able to change the window
			fprintf(gp, "unset multiplot\n");
			// Unset plot size 
			fprintf(gp, "unset size\n");
			
			/* --- Plot ground truth velocity and estimated velocity with state estimation error --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			// Set multiplot mode: plot multiple plots in one graph
			fprintf(gp, "set multiplot\n");
			// Adjust graph size for 2 x 1 subplots
			fprintf(gp, "set size 1, 0.5\n");

			fprintf(gp, "set origin 0.0, 0.5\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'instantaneous velocity (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'velocity in [m/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader, '' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 4,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 12);
		
			fprintf(gp, "set origin 0.0, 0.0\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'instantaneous velocity state estimation error (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'estimation error in [m/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 19);
			
			// Unset multiplot mode to be able to change the window
			fprintf(gp, "unset multiplot\n");
			// Unset plot size 
			fprintf(gp, "unset size\n");

			/* --- Plot ground truth yaw angle and estimated yaw angle with state estimation error --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			// Set multiplot mode: plot multiple plots in one graph
			fprintf(gp, "set multiplot\n");
			// Adjust graph size for 2 x 1 subplots
			fprintf(gp, "set size 1, 0.5\n");

			fprintf(gp, "set origin 0.0, 0.5\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'yaw angle (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'yaw angle in [rad]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader, '' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 5,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 13);
		
			fprintf(gp, "set origin 0.0, 0.0\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'yaw angle state estimation error (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'estimation error in [rad]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 20);
			
			// Unset multiplot mode to be able to change the window
			fprintf(gp, "unset multiplot\n");
			// Unset plot size 
			fprintf(gp, "unset size\n");

			/* --- Plot ground truth yaw rate and estimated yaw rate with state estimation error --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			// Set multiplot mode: plot multiple plots in one graph
			fprintf(gp, "set multiplot\n");
			// Adjust graph size for 2 x 1 subplots
			fprintf(gp, "set size 1, 0.5\n");

			fprintf(gp, "set origin 0.0, 0.5\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'yaw rate (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'yaw rate in [rad/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader, '' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 6,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 14);
		
			fprintf(gp, "set origin 0.0, 0.0\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'yaw rate state estimation error (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'estimation error in [rad/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 21);
			
			// Unset multiplot mode to be able to change the window
			fprintf(gp, "unset multiplot\n");
			// Unset plot size 
			fprintf(gp, "unset size\n");
			
			/* --- Plot ground truth x-velocity and estimated x-velocity with state estimation error --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			// Set multiplot mode: plot multiple plots in one graph
			fprintf(gp, "set multiplot\n");
			// Adjust graph size for 2 x 1 subplots
			fprintf(gp, "set size 1, 0.5\n");

			fprintf(gp, "set origin 0.0, 0.5\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'x-velocity (%s) relative to ego car'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'x-velocity in [m/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader, '' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 7,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 15);
		
			fprintf(gp, "set origin 0.0, 0.0\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'x-velocity state estimation error (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'estimation error in [m/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 22);
			
			// Unset multiplot mode to be able to change the window
			fprintf(gp, "unset multiplot\n");
			// Unset plot size 
			fprintf(gp, "unset size\n");
			
			/* --- Plot ground truth y-velocity and estimated y-velocity with state estimation error --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			// Set multiplot mode: plot multiple plots in one graph
			fprintf(gp, "set multiplot\n");
			// Adjust graph size for 2 x 1 subplots
			fprintf(gp, "set size 1, 0.5\n");

			fprintf(gp, "set origin 0.0, 0.5\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'y-velocity (%s) relative to ego car'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'y-velocity in [m/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader, '' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 8,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 16);
		
			fprintf(gp, "set origin 0.0, 0.0\n");
			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'y-velocity state estimation error (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'estimation error in [m/s]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 23);		

			// Unset multiplot mode to be able to change the window
			fprintf(gp, "unset multiplot\n");
			// Unset plot size 
			fprintf(gp, "unset size\n");
			
			/* --- Plot ground truth acceleration --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'instantaneous acceleration (%s) relative to ego car'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'acceleration in [m/s^2]'\n");
			fprintf(gp,
			    "plot '%s' using %d:%d with lines title columnheader\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 9);
			
			/* --- Plot NIS (Normalized Innovation Sqaured) values for Lidar and Radar --- */

			// Create new plot
			fprintf(gp, "set terminal qt %d\n", plot_no);
			// Increment plot number
			plot_no++;

			fprintf(gp, "set grid\n");
			fprintf(gp, "set title 'NIS - Normalized Innovation Squared (%s)'\n", car_name);
			fprintf(gp, "set xlabel 'time in [µs]'\n");
			fprintf(gp, "set ylabel 'NIS'\n");
			fprintf(gp, 
				"plot '%s' using %d:%d with lines title 'NIS_L_i_d_a_r', '' using %d:%d with lines title 'NIS_R_a_d_a_r'\n",
				data_log_filepath,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 24,
				i * num_of_cols_per_car + 1,
				i * num_of_cols_per_car + 25);
		}
        fclose(gp);
	}
};