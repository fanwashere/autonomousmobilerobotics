#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include <boost/bind.hpp>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>

#define UPDATE_RATE 10 // Hz
#define GRID_RESOLUTION 0.1 // m/cell
#define GRID_WIDTH 100 // # of cells
#define GRID_HEIGHT 100 // # of cells

typedef const boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)> poseCallback;
typedef const boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)> rangeCallback;

/* Position Helper Code */

class Pose {
	public:
		float x;
		float y;
		float theta;
};

class poseHandler {
	private:
		float x;
		float y;
		float theta;
	public:
		void callback(const gazebo_msgs::ModelStates::ConstPtr&);
		Pose getPose();
};

void poseHandler::callback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
	this->x = msg->pose[1].position.x;
	this->y = msg->pose[1].position.y;

	tf::Quaternion q(msg->pose[1].orientation.x, msg->pose[1].orientation.y, msg->pose[1].orientation.z, msg->pose[1].orientation.w);
	q.normalize();
	const tf::Matrix3x3 mat(q);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	this->theta = (float)yaw;

	// ROS_INFO("Model States: [%f %f]", this->x, this->y);
}

Pose poseHandler::getPose()
{
	Pose pose;
	pose.x = this->x;
	pose.y = this->y;
	pose.theta = this->theta;

	return pose;
}

/* Range Scan Helper Code */

class Scan {
	public:
		float angle;
		float range;
};

class rangeHandler {
	private:
		std::vector<Scan> ranges;
	public:
		void callback(const sensor_msgs::LaserScan::ConstPtr&);
		std::vector<Scan> getRanges();
};

void rangeHandler::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int i, numMeasurements = msg->ranges.size();
	float currentAngle = msg->angle_min;
	std::vector<Scan> ranges;

	for (i = 0; i < numMeasurements; i++) {
		if (isnan(msg->ranges[i]) || msg->ranges[i] < msg->range_min || msg->ranges[i] > msg->range_max) {
			currentAngle += msg->angle_increment;
			continue;
		} else {
			Scan scan;
			scan.angle = currentAngle;
			scan.range = msg->ranges[i];

			ranges.push_back(scan);
			currentAngle += msg->angle_increment;
		}
	}

	this->ranges = ranges;

	// ROS_INFO("Range: %f", this->range);
}

std::vector<Scan> rangeHandler::getRanges()
{
	return this->ranges;
}

/* Helper Functions */

float logit(float p) {
	return log(p) - log(1 - p);
}

int logit_int(float p) {
	return (int)round(logit(p) * 100);
}

/* Node Main */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mapper_node");
	ros::NodeHandle n;

	poseHandler posHandler;
	poseCallback poseSubscriberCallback = boost::bind(&poseHandler::callback, &posHandler, _1);

	rangeHandler rngHandler;
	rangeCallback rangeSubscriberCallback = boost::bind(&rangeHandler::callback, &rngHandler, _1);

	ros::Subscriber poseSubscriber = n.subscribe("/gazebo/model_states", 1, poseSubscriberCallback);
	ROS_INFO("Subscribed to /gazebo/model_states topic.");

	ros::Subscriber rangeSubscriber = n.subscribe("/scan", 1, rangeSubscriberCallback);
	ROS_INFO("Subscribed to /scan topic.");

	ros::Publisher gridPublisher = n.advertise<nav_msgs::OccupancyGrid>("grid", 10);

	ros::Rate loop_rate(UPDATE_RATE);

	// Initialize Occupancy Grid
	int grid[GRID_WIDTH * GRID_HEIGHT] = {logit_int(0.5)};

	nav_msgs::OccupancyGrid msg;
	msg.info.resolution = (float)GRID_RESOLUTION;
	msg.info.width = (int)GRID_WIDTH;
	msg.info.height = (int)GRID_HEIGHT;

	std::vector<signed char> gridVector(grid, grid + (GRID_WIDTH * GRID_HEIGHT));
	msg.data = gridVector;

	while(ros::ok())
	{
		const Pose pose = posHandler.getPose();
		const std::vector<Scan> ranges = rngHandler.getRanges();

		const int gridX = (pose.x / GRID_RESOLUTION) + (GRID_WIDTH / 2);
		const int gridY = (pose.y / GRID_RESOLUTION) + (GRID_HEIGHT / 2);

		ROS_INFO("Position Abs[%f %f %f] Grid[%d %d]", pose.x, pose.y, pose.theta, gridX, gridY);

		if (ranges.size() > 0) {
			int i;
			for (i = 0; i < ranges.size(); i++) {
				const float angle = ranges[i].angle + pose.theta + M_PI;
				const int hitX = (int)round(ranges[i].range * sin(angle) / GRID_RESOLUTION) + gridX;
				const int hitY = (int)round(ranges[i].range * -cos(angle) / GRID_RESOLUTION) + gridY;

				//ROS_INFO("Laser Scan [%f %f] [%d %d]", ranges[i].range, ranges[i].angle, hitX, hitY);
				grid[hitY * GRID_WIDTH + hitX] = 100;
				// ROS_INFO("Scan [%d %d] [%d]", hitX, hitY, hitX * GRID_WIDTH + hitY);
			}
		}

		ROS_INFO("Processed %d scans.", ranges.size());

		std::vector<signed char> newGridVector(grid, grid + (GRID_WIDTH * GRID_HEIGHT));
		msg.data = newGridVector;
		gridPublisher.publish(msg);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
