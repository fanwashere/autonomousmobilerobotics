#include "state_estimation.h"

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

namespace
{
    const double RATE = 1.0;

    typedef boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)> PoseSimCallback;
    typedef boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> PoseLiveCallback;
    typedef boost::function<void(const nav_msgs::Odometry::ConstPtr&)> OdometryCallback;
}

Pose PoseHandler::getPose() const
{
    return pose;
}

void PoseHandler::callbackSim(const gazebo_msgs::ModelStates::ConstPtr msg)
{
    int i;
    for(i = 0; i < msg->name.size(); i++) if(msg->name[i] == "mobile_base") break;

    pose.x = msg->pose[i].position.x ;
    pose.y = msg->pose[i].position.y ;
    pose.yaw = tf::getYaw(msg->pose[i].orientation);
}

void PoseHandler::callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf::getYaw(msg->pose.pose.orientation);
}

Odometry OdometryHandler::getOdometry() const
{
    return odometry;
}

void OdometryHandler::callback(const nav_msgs::Odometry::ConstPtr msg)
{
    odometry.pose = msg->pose;
    odometry.twist = msg->twist;
}

void publishParticles(const ros::Publisher& publisher, const Pose& pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/particle_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "particle_filter";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.color.g = 1.0f;

    geometry_msgs::Point point;
    point.x = pose.x;
    point.y = pose.y;

    marker.points.push_back(point);

    publisher.publish(marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimation");
    ros::NodeHandle n;
    ros::Rate rate(RATE);

    PoseHandler poseHandler;
#ifdef LIVE
    PoseLiveCallback poseLiveCallback = boost::bind(&PoseHandler::callbackLive, &poseHandler, _1);
    ros::Subscriber poseLiveSubscriber = n.subscribe("/indoor_pos", 1, poseLiveCallback);
    ROS_INFO("Subscribed to /indoor_pos topic");
#else
    PoseSimCallback poseSimCallback = boost::bind(&PoseHandler::callbackSim, &poseHandler, _1);
    ros::Subscriber poseSimSubscriber = n.subscribe("/gazebo/model_states", 1, poseSimCallback);
    ROS_INFO("Subscribed to /gazebo/model_states topic");
#endif

    OdometryHandler odomHandler;
    OdometryCallback odomCallback = boost::bind(&OdometryHandler::callback, &odomHandler, _1);
    ros::Subscriber odomSubscriber = n.subscribe("/odom", 1, odomCallback);
    ROS_INFO("Subscribed to /odom topic");

    ros::Publisher particlePublisher = n.advertise<visualization_msgs::Marker>("/particle_filter", 1);

    while(ros::ok())
    {
        const Odometry odom = odomHandler.getOdometry();
        const Pose pose = poseHandler.getPose();

        publishParticles(particlePublisher, pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}