#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>

// #define LIVE 

struct Pose 
{
    double x;
    double y;
    double yaw;
};

class PoseHandler
{
public:
    PoseHandler() = default;
    virtual ~PoseHandler() = default;

    Pose getPose() const;
    void callbackSim(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

private:
    Pose pose;
};