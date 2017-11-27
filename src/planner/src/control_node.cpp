#include <ros/ros.h>
#include "pose.h"
#include "map.h"
#include "visualizer.h"
#include <visualization_msgs/Marker.h>

namespace {
    const std::string NODE_NAME = "controller";
    const double RATE = 10.0;

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

    ros::Publisher visualizationPublisher = n.advertise<visualization_msgs::Marker>("robot", 1);
    Visualizer visualizer(visualizationPublisher);
    visualizer.setScalingFactor(1.0); // Since pose is given in actual meters, don't need scaling.

    ros::Rate loop_rate(RATE);

    while (ros::ok()) {
        Pose pose = poseHandler.getPose();
        Coordinate coord(pose.x, pose.y);
        visualizer.drawRobot(coord, pose.yaw);

    	loop_rate.sleep();
    	ros::spinOnce();
    }

    return 0;
}
