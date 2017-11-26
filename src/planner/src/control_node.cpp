#include <ros/ros.h>
#include "path.h"
#include "map.h"
#include "pose.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

namespace {
    const std::string NODE_NAME = "controller";
    const double RATE = 10.0;

    using PathCallback = boost::function<void(const nav_msgs::Path::ConstPtr&)>;
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
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
        ROS_INFO("Waiting on map data, sleeping for 500ms...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    
    while (ros::ok()) {
    	loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
