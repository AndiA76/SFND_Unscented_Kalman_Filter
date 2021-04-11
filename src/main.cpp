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

// Unscented Kalman Filter Highway Project implementation for object tracking

// Create a simple 3D highway enviroment with traffic to be tracked using PCL

//#include "render/render.h"
#include "highway.h"


int main(int argc, char** argv)
{
	// Set up the viewer to visualize the highway scenario
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// Set camera position and camera orientation angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	// Initialize simple 3D highway scenario with traffic
	Highway highway(viewer);

	//initHighway(viewer);

	// Set general simulation parameters
	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	// Set ego velocity
	double egoVelocity = 25;

	// Initialize data log
	highway.initDataLog();

	// Loop over all frames, simulate the scenes for each frame and log the tracking data
	while (frame_count < (frame_per_sec*sec_interval))
	{
		// Refresh the scene by removing all point clouds and shapes
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		// Generate the next scene
		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000/frame_per_sec);

		// Increment frame count
		frame_count++;

		// Increment simualtion time
		time_us = 1000000*frame_count/frame_per_sec;		
	}

	// Close data log
	highway.closeDataLog();

	// Plot log data
	highway.plotLogData();
}