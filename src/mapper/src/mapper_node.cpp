#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>
#include <boost/bind.hpp>
#include <tf/transform_datatypes.h>

#define UPDATE_RATE 10 // Hz
#define GRID_RESOLUTION 0.1 // m/cell
#define GRID_WIDTH 100 // # of cells
#define GRID_HEIGHT 100 // # of cells
#define GRID_DEFAULT_PROBABILITY 0.5 // Grid probability at initialization
#define P_NO_OBJECT 0.4 // Probability assigned if no hit in the cell
#define P_HIT_OBJECT 0.6 // Probability assigned if hit something in the cell

#define sign(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))

typedef const boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)> PoseCallback;
typedef const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> PoseCallbackLive;
typedef const boost::function<void(const sensor_msgs::LaserScan::ConstPtr&)> RangeCallback;

/* Position Helper Code */

class Pose {
 public:
  float x;
  float y;
  float theta;
};

class PoseHandler {
 private:
  float x;
  float y;
  float theta;
 public:
  void callback(const gazebo_msgs::ModelStates::ConstPtr&);
  void callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&);
  Pose getPose();
};

void PoseHandler::callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  this->x = msg->pose[1].position.x;
  this->y = msg->pose[1].position.y;

  tf::Quaternion q(msg->pose[1].orientation.x, msg->pose[1].orientation.y, msg->pose[1].orientation.z, msg->pose[1].orientation.w);
  q.normalize();
  const tf::Matrix3x3 mat(q);
  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);
  this->theta = (float)yaw;
}

void PoseHandler::callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  this->x = msg->pose.pose.position.x;
  this->y = msg->pose.pose.position.y;
  this->theta = tf::getYaw(msg->pose.pose.orientation);
}

Pose PoseHandler::getPose() {
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

class RangeHandler {
 private:
  std::vector<Scan> ranges;
 public:
  void callback(const sensor_msgs::LaserScan::ConstPtr&);
  std::vector<Scan> getRanges();
};

void RangeHandler::callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
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

std::vector<Scan> RangeHandler::getRanges() {
  return this->ranges;
}

/* Helper Functions */

float logit(float p) {
  return log(p) - log(1 - p);
}

float logitToProbability(float pLogit) {
  return (int)(100.0 * exp(pLogit) / (1 + exp(pLogit)));
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
  void update(int x, int y, float p);
  Grid(int width, int height);
  Grid(int width, int height, float defaultProbability);
 private:
  std::vector<float> pGrid;
  std::vector<signed char> grid;
  int width;
  int height;
};

Grid::Grid(int width, int height) {
  this->width = width;
  this->height = height;
  std::vector<float> pGrid(width * height, logit(0));
  std::vector<signed char> grid(width * height, logitToProbability(logit(0)));
  this->pGrid = pGrid;
  this->grid = grid;
}

Grid::Grid(int width, int height, float defaultProbability) {
  this->width = width;
  this->height = height;
  std::vector<float> pGrid(width * height, logit(defaultProbability));
  std::vector<signed char> grid(width * height, logitToProbability(logit(defaultProbability)));
  this->pGrid = pGrid;
  this->grid = grid;
}

std::vector<signed char> Grid::getVector() {
  return this->grid;
}

void Grid::update(int x, int y, float p) {
  const int index = y * this->width + x;
  this->pGrid[index] += logit(p) - logit(GRID_DEFAULT_PROBABILITY);
  this->grid[index] = logitToProbability(this->pGrid[index]);
}

/* Node Main */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper_node");
  ros::NodeHandle n;

  PoseHandler poseHandler;
  RangeHandler rangeHandler;

#ifdef LIVE
  PoseCallbackLive poseSubscriberCallbackLive = boost::bind(&PoseHandler::callbackLive, &poseHandler, _1);
  ros::Subscriber poseSubscriber = n.subscribe("/indoor_pos", 1, poseSubscriberCallbackLive);
  ROS_INFO("Subscribed to /indoor_pos topic.");
#else
  PoseCallback poseSubscriberCallback = boost::bind(&PoseHandler::callback, &poseHandler, _1);
  ros::Subscriber poseSubscriber = n.subscribe("/gazebo/model_states", 1, poseSubscriberCallback);
  ROS_INFO("Subscribed to /gazebo/model_states topic.");
#endif

  RangeCallback rangeSubscriberCallback = boost::bind(&RangeHandler::callback, &rangeHandler, _1);
  ros::Subscriber rangeSubscriber = n.subscribe("/scan", 1, rangeSubscriberCallback);
  ROS_INFO("Subscribed to /scan topic.");

  ros::Publisher gridPublisher = n.advertise<nav_msgs::OccupancyGrid>("grid", 10);

  ros::Rate loop_rate(UPDATE_RATE);

  // Initialize Occupancy Grid
  Grid grid(GRID_WIDTH, GRID_HEIGHT, GRID_DEFAULT_PROBABILITY);

  // Initialize Occupancy Grid Message
  nav_msgs::OccupancyGrid msg;
  msg.info.resolution = (float)GRID_RESOLUTION;
  msg.info.width = (int)GRID_WIDTH;
  msg.info.height = (int)GRID_HEIGHT;
  msg.data = grid.getVector();

  while(ros::ok()) {
    const Pose pose = poseHandler.getPose();
    const std::vector<Scan> ranges = rangeHandler.getRanges();

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
          grid.update(coord.x, coord.y, P_NO_OBJECT);
        }

        grid.update(hitX, hitY, P_HIT_OBJECT);
      }
    }

    msg.data = grid.getVector();
    gridPublisher.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
