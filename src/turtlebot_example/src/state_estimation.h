#pragma once

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>

struct Pose 
{
    double x;
    double y;
    double yaw;
};

class PoseHandler
{
public:
    Pose getPose() const;
    void callbackSim(const gazebo_msgs::ModelStates::ConstPtr msg);
    void callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);

private:
    Pose pose;
};

struct Odometry
{
    geometry_msgs::PoseWithCovariance pose;
    geometry_msgs::TwistWithCovariance twist;
};

class OdometryHandler
{
public:
    Odometry getOdometry() const;
    void callback(const nav_msgs::Odometry::ConstPtr msg);

private:
    Odometry odometry;
};