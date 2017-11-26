#include <ros/ros.h>
#include "control_node.h"
#include "path.h"
#include "map.h"
#include "pose.h"
#include <tf/transform_datatypes.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

namespace {
    const std::string NODE_NAME = "controller";
    const double RATE = 10.0;

    std::random_device rngDevice;  // Will be used to obtain a seed for the random number engine
    std::mt19937 rngGenerator(rngDevice()); //Standard mersenne_twister_engine seeded with random_device()
    std::uniform_real_distribution<> realDist(-1.0, 1.0);
    std::normal_distribution<> noise(-0.1, 0.1);


    using PathCallback = boost::function<void(const nav_msgs::Path::ConstPtr&)>;
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
}

Pose PoseHandler::getPose() const
{
    return pose;
}

void PoseHandler::callbackSim(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    int i;
    for(i = 0; i < msg->name.size(); i++) if(msg->name[i] == "mobile_base") break;

    pose.x = msg->pose[i].position.x + noise(rngGenerator);
    pose.y = msg->pose[i].position.y + noise(rngGenerator);
    pose.yaw = tf::getYaw(msg->pose[i].orientation) + noise(rngGenerator);
}

void PoseHandler::callbackLive(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.yaw = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

#ifdef LIVE
    PoseHandlerLive poseHandler;
    PoseLiveCallback poseLiveCallback = boost::bind(&PoseHandlerLive::callback, &poseHandler, _1);
    auto poseLiveSubscriber = n.subscribe("/indoor_pos", 1, poseLiveCallback);
    ROS_INFO("Subscribed to /indoor_pos topic");
#else
    PoseHandlerSim poseHandler;
    PoseSimCallback poseSimCallback = boost::bind(&PoseHandlerSim::callback, &poseHandler, _1);
    auto poseSimSubscriber = n.subscribe("/gazebo/model_states", 1, poseSimCallback);
    ROS_INFO("Subscribed to /gazebo/model_states topic");
#endif

    PathHandler pathHandler;
    PathCallback pathCallback = boost::bind(&PathHandler::callback, &pathHandler, _1);
    auto pathSubscriber = n.subscribe("/path", 1, pathCallback);
    ROS_INFO("Subscribed to /path topic");

    ros::Rate loop_rate(RATE);

    // Wait for path to become available.
    while(ros::ok() && !pathHandler.hasPath()) {
        ROS_INFO("Waiting on path data, sleeping for 1s...");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    
    // Get the entire shortest path to traverse
    std::shared_ptr<Path> path = pathHandler.getPath();
    
    while (ros::ok()) {
    	loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
