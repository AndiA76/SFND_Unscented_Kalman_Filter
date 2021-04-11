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

// Functions and structs used to render the enviroment such as highway and cars

#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <vector>
#include <string>
#include "../ukf.h"

// Structure to hold the color settting
struct Color
{
	float r, g, b;

	Color(float setR, float setG, float setB)
		: r(setR), g(setG), b(setB)
	{}
};

// Structure to hold a 3D vector (incl. 3D vector addition)
struct Vect3
{
	double x, y, z;

	Vect3(double setX, double setY, double setZ)
		: x(setX), y(setY), z(setZ)
	{}

	Vect3 operator+(const Vect3& vec)
	{
		Vect3 result(x + vec.x, y + vec.y, z + vec.z);
		return result;
	}
};

// Camera orientation setting
enum CameraAngle
{
	XY, TopDown, Side, FPS
};

// Structure to hold an actuation command
struct accuation
{
	long long time_us;
	float acceleration;
	float steering;

	accuation(long long t, float acc, float s)
		: time_us(t), acceleration(acc), steering(s)
	{}
};

// Structure to hold a car object in the virtual simulation environment
struct Car
{
	// units in meters
	Vect3 position, dimensions;
	Eigen::Quaternionf orientation;
	std::string name;
	Color color;
	// instantaneous velocity
	float velocity;
	// yaw angle
	float angle;
	// instantaneous acceleration
	float acceleration;
	// steering angle
	float steering;
	// distance between front/rear (axle) of vehicle and center of gravity
	float Lf;
	// yaw rate
	float yaw_rate;

	// Unscented Kalman Filter
	UKF ukf;

	// accuation instructions
	std::vector<accuation> instructions;
	int accuateIndex;

	double sinNegTheta;
	double cosNegTheta;

	// Initialize a new car object without arguments (default setting)
	Car()
		: position(Vect3(0,0,0)), dimensions(Vect3(0,0,0)), color(Color(0,0,0))
	{}

	// Initialize a new car object with arguments
	Car(Vect3 setPosition, Vect3 setDimensions, Color setColor, float setVelocity, float setAngle, float setLf, std::string setName)
		: position(setPosition), dimensions(setDimensions), color(setColor), velocity(setVelocity), angle(setAngle), Lf(setLf), name(setName)
	{
		orientation = getQuaternion(angle);
		acceleration = 0;
		steering = 0;
		yaw_rate = 0;
		accuateIndex = -1;

		sinNegTheta = sin(-angle);
		cosNegTheta = cos(-angle);
	}

	// Quaternion for rotation around the z axis by an angle theta
	Eigen::Quaternionf getQuaternion(float theta)
	{
		Eigen::Matrix3f rotation_mat;
  		rotation_mat << 
  		cos(theta), -sin(theta), 0,
    	sin(theta),  cos(theta), 0,
    	0, 			 0, 		 1;
    	
		Eigen::Quaternionf q(rotation_mat);
		return q;
	}

	// Render a car model made of two cuboids for bottom and top 
	void render(pcl::visualization::PCLVisualizer::Ptr& viewer)
	{
		// Render bottom of car
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*1/3), orientation, dimensions.x, dimensions.y, dimensions.z*2/3, name+"frame");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"frame");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"frame");
		
		// Render top of car
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Top");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name + "Top");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "Top");
		viewer->addCube(Eigen::Vector3f(position.x, position.y, dimensions.z*5/6), orientation, dimensions.x/2, dimensions.y, dimensions.z*1/3, name + "Topframe");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name+"Topframe");
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, name+"Topframe");
	}

	// Set resulting acceleration in [m/s^2]
	void setAcceleration(float setAcc)
	{
		acceleration = setAcc;
	}

	// Set steering angle in [rad]
	void setSteering(float setSteer)
	{
		steering = setSteer;
	}

	// Update yaw rate in [rad/s] using bicylce model
	void updateYawRate()
	{
		// Udate yaw rate from instantaneous velocity, the distance between front/rear axle and 
		// the instantaneous steering angle using a simple bicyle model
		if (steering == 0)
		{
			// Set yaw rate to zero if there is no steering command
			yaw_rate = 0;
		}
		else
		{
			// Radius about instantaneous center of rotation: R = 2 * Lf / tan(steering)

			// Yaw rate about instantaneous center of rotation
			yaw_rate = velocity * tan(steering) / (2 * Lf);
		}
	}

	// Set actuation instructions for a car
	void setInstructions(std::vector<accuation> setIn)
	{
		for(accuation a : setIn)
			instructions.push_back(a);
	}

	// Set Unscented Kalman Filter for a car
	void setUKF(UKF tracker)
	{
		ukf = tracker;
	}

	// Move the car according to its instantaneous state and actuation instructions
	void move(float dt, int time_us)
	{
		// Set motion changes to be considered in this time step
		if(instructions.size() > 0 && accuateIndex < (int)instructions.size()-1)
		{
			if(time_us >= instructions[accuateIndex+1].time_us)
			{
				setAcceleration(instructions[accuateIndex+1].acceleration);
				setSteering(instructions[accuateIndex+1].steering);
				updateYawRate();
				accuateIndex++;
			}
		}

		// Update actual state of the car (ground truth)
		position.x += velocity * cos(angle) * dt;
		position.y += velocity * sin(angle) * dt;
		angle += velocity*steering*dt/Lf;
		orientation = getQuaternion(angle);
		velocity += acceleration*dt;

		// Calculate sin() and cos() of the car's yaw angle
		sinNegTheta = sin(-angle);
		cosNegTheta = cos(-angle);
	}

	// Collision helper function
	bool inbetween(double point, double center, double range)
	{
		return (center - range <= point) && (center + range >= point);
	}

	// Check for collisions
	bool checkCollision(Vect3 point)
	{
		// Check collision for rotated car
		double xPrime = ((point.x-position.x) * cosNegTheta - (point.y-position.y) * sinNegTheta)+position.x;
		double yPrime = ((point.y-position.y) * cosNegTheta + (point.x-position.x) * sinNegTheta)+position.y;

		return (inbetween(xPrime, position.x, dimensions.x / 2) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
			(inbetween(xPrime, position.x, dimensions.x / 4) && inbetween(yPrime, position.y, dimensions.y / 2) && inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));
	}
};

// Rendering functions
void renderHighway(double distancePos, pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderRays(pcl::visualization::PCLVisualizer::Ptr& viewer, const Vect3& origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
void clearRays(pcl::visualization::PCLVisualizer::Ptr& viewer);
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1, 1, 1));
void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1, -1, -1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1, 0, 0), float opacity = 1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, BoxQ box, int id, Color color = Color(1, 0, 0), float opacity = 1);

#endif
