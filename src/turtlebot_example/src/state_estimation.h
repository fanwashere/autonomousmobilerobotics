#pragma once

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Point.h>
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
    PoseHandler() = default;
    virtual ~PoseHandler() = default;

    Pose getPose() const;
    void callbackSim(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

private:
    Pose pose;
};

struct Odometry
{
    Pose pose;
    geometry_msgs::TwistWithCovariance twist;
};

class OdometryHandler
{
public:
    OdometryHandler() = default;
    virtual ~OdometryHandler() = default;

    Odometry getOdometry() const;
    void callback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    Odometry odometry;
};

class ParticleFilter
{
public:
    ParticleFilter() = delete;
    explicit ParticleFilter(uint32_t numParticles);
    virtual ~ParticleFilter() = default;

    void run(const Pose& newPose);

private:
    ros::Time time;
    uint32_t numParticles;
    std::vector<double> weights;
    std::vector<Pose> particles;
    std::vector<Pose> predictions;
};