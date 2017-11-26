#include <ros/ros.h>
#include "pose.h"
#include "path.h"
#include "map.h"
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

namespace {
    const std::string NODE_NAME = "controller";
    const double RATE = 10.0;
    const double POSITION_ERROR = 0.001;

    using PathCallback = boost::function<void(const nav_msgs::Path::ConstPtr&)>;
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;

    bool hasReachedNode(Pose pose, Node node) {
        double xerr = abs(pose.x - node.x);
        double yerr = abs(pose.y - node.y);

        return (xerr <= POSITION_ERROR && yerr <= POSITION_ERROR);
    }

    void getLine(float x1, float y1, float x2, float y2, float &a, float &b, float &c) {
        a = y2 - y1;
        b = x2 - x1;
        c = x1*y2 - x2*y1;
    }

    bool distanceToLineSegment(Node p1, Node p2, Pose x, float &crossTrackError) {
        bool outside = false;

        Node line_segment = Node( p2.x - p1.x, p2.y - p1.y );
        Node p1_to_x = Node(x.x - p1.x, x.y - p1.y);

        float a, b, c;
        getLine(p1.x, p1.y, p2.x, p2.y, a, b, c);
        float distance = abs( a*x.x + b*x.y + c )/ sqrt(a*a + b*b);

        float norm = a*a + b*b;
        float dot_line_segment_p1_to_x = line_segment.x*p1_to_x.x + line_segment.y*p1_to_x.y;

        if (dot_line_segment_p1_to_x/norm > 1) {
            outside = true;
        }

        

        return outside;
    }
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
    Path pathObject = pathHandler.getPath();
    std::vector<Node> path = pathObject.path;
    
    for (int i = 0; i < pathObject.getTotalNodes() - 1; ++i) {
        Node end_point = path[i+1];
        Node start_point = path[i];
        // traj_angle =  rads
        double traj_angle = atan2( end_point.y - start_point.y, end_point.x - start_point.x );

        Pose currentPose = poseHandler.getPose();

        while (!hasReachedNode(currentPose, end_point)) {
            // Add the controller and make it travel for a few seconds
        }

    }
    
    while (ros::ok()) {
    	loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
