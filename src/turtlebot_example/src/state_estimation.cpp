#include "state_estimation.h"

#include <ros/ros.h>
#include <boost/bind.hpp>
#include <tf/transform_datatypes.h>

typedef const boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)> PoseSimCallback;
typedef const boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> PoseLiveCallback;
typedef const boost::function<void(const nav_msgs::Odometry::ConstPtr&)> OdometryCallback;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_estimation");
    ros::NodeHandle n;

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

    ros::Rate rate(1);

    while(ros::ok())
    {
        const Odometry odom = odomHandler.getOdometry();
        const Pose pose = poseHandler.getPose();

        ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom.pose.pose.position.x,odom.pose.pose.position.y, odom.pose.pose.position.z);
        ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom.twist.twist.linear.x,odom.twist.twist.angular.z);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}