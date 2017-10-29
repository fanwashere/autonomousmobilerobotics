#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include <boost/bind.hpp>
#include <tf/transform_datatypes.h>

#define UPDATE_RATE 10 // Hz
#define GRID_RESOLUTION 0.1 // m/cell
#define GRID_WIDTH 100 // # of cells
#define GRID_HEIGHT 100 // # of cells

#define sign(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))

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

void poseHandler::callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  this->x = msg->pose[1].position.x;
  this->y = msg->pose[1].position.y;

  tf::Quaternion q(msg->pose[1].orientation.x, msg->pose[1].orientation.y, msg->pose[1].orientation.z, msg->pose[1].orientation.w);
  q.normalize();
  const tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  this->theta = (float)yaw;
}

Pose poseHandler::getPose() {
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

void rangeHandler::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
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
}

std::vector<Scan> rangeHandler::getRanges() {
  return this->ranges;
}

/* Helper Functions */

float logit(float p) {
  return log(p) - log(1 - p);
}

int logit_int(float p) {
  return (int)round(logit(p) * 100);
}

class Coordinate {
 public:
  int x;
  int y;
  Coordinate(int x, int y);
};

Coordinate::Coordinate(int x, int y) {
  this->x = x;
  this->y = y;
}

std::vector<Coordinate> bresenham(int x0, int y0, int x1, int y1) {
  std::vector<Coordinate> lineCoordinates;
  int x = x0;
  int y = y0;

  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);

  int s1 = sign(x1 - x0);
  int s2 = sign(y1 - y0);

  int swap = 0;
  if (dy > dx) {
    int temp = dx;
    dx = dy;
    dy = temp;
    swap = 1;
  }

  int D = 2 * dy - dx;
  int i;
  for (i = 0; i < dx; i++) {
    Coordinate coord(x, y);
    lineCoordinates.push_back(coord);

    while (D >= 0) {
      D = D - 2 * dx;
      if (swap) {
        x += s1;
      } else {
        y += s2;
      }
    }

    D = D + 2 * dy;
    if (swap) {
      y += s2;
    } else {
      x += s1;
    }
  }

  return lineCoordinates;
}

class Grid {
 public:
  std::vector<signed char> getVector();
  void update(int x, int y, int p);
  Grid(int width, int height);
  Grid(int width, int height, int defaultValue);
 private:
  std::vector<signed char> grid;
  int width;
  int height;
};

Grid::Grid(int width, int height) {
  this->width = width;
  this->height = height;
  std::vector<signed char> grid(width * height, 0);
  this->grid = grid;
}

Grid::Grid(int width, int height, int defaultValue) {
  this->width = width;
  this->height = height;
  std::vector<signed char> grid(width * height, defaultValue);
  this->grid = grid;
}

std::vector<signed char> Grid::getVector() {
  return this->grid;
}

void Grid::update(int x, int y, int p) {
  this->grid[y * this->width + x] = p;
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
  Grid grid(GRID_WIDTH, GRID_HEIGHT, 50);

  // Initialize Occupancy Grid Message
  nav_msgs::OccupancyGrid msg;
  msg.info.resolution = (float)GRID_RESOLUTION;
  msg.info.width = (int)GRID_WIDTH;
  msg.info.height = (int)GRID_HEIGHT;
  msg.data = grid.getVector();

  while(ros::ok()) {
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

        if (hitX > GRID_WIDTH || hitX < 0 || hitY > GRID_HEIGHT || hitY < 0) {
          continue;
        }

        const std::vector<Coordinate> lineCoordinates = bresenham(gridX, gridY, hitX, hitY);

        int j;
        for (j = 0; j < lineCoordinates.size(); j++) {
          Coordinate coord = lineCoordinates[j];
          grid.update(coord.x, coord.y, 0);
        }

        grid.update(hitX, hitY, 100);
      }
    }

    msg.data = grid.getVector();
    gridPublisher.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();
  return 0;
}
