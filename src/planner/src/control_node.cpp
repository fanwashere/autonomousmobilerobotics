#include <ros/ros.h>
#include "pose.h"
#include "path.h"
#include "map.h"
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <Eigen/Dense>

using namespace Eigen;

namespace {
    const std::string NODE_NAME = "controller";
    const double RATE = 10.0;
    
    const float pi = 3.141592;
    
    const float headingGain = 0.75;
    const float crosstrackGain = 1;
    
    const float velocity = 0.2;
    const float delta_max = 25*pi/180;
    const float robot_length = 0.25;

    using PathCallback = boost::function<void(const nav_msgs::Path::ConstPtr&)>;
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
    
    float angleWrap(float angle) {
        return std::fmod(angle+pi, 2*pi) - pi;
    }

    bool distanceToLineSegment(Vector2f p1, Vector2f p2, Vector2f x, float &crosstrackError) {
        bool outside = false;

        if ( (p2-x).norm() < 0.5) {
            outside = true;
        }
        
        Vector2f line_segment = p2-p1;
        Vector2f p1_to_x = x-p1;

        Vector2f projection = p1 + (line_segment.dot(p1_to_x) / line_segment.squaredNorm()) * line_segment;
        Vector2f distance = x - projection;

        ROS_INFO("THRESHOLD VALUE  -> %f", (line_segment.dot(p1_to_x) / line_segment.squaredNorm()));

        // if ( (line_segment.dot(p1_to_x) / line_segment.squaredNorm())  > 1 ) {
        //     outside = true;
        // }

        float pos_neg = 1.0;
        
        Vector3f updated_line_segment(line_segment(0), line_segment(1), 0);
        Vector3f updated_p1_to_x(p1_to_x(0), p1_to_x(1), 0);
        
        // ---- BE CAREFUL HERE ----- //
        Vector3f cross_product = updated_line_segment.cross(updated_p1_to_x);

        if (cross_product(2) < 0) {
            crosstrackError = -1.0*distance.norm();
        } else {
            crosstrackError = 1.0*distance.norm();
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

    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;

    ros::Rate loop_rate(RATE);

    // Wait for path to become available.
    while(ros::ok() && !pathHandler.hasPath()) {
        ROS_INFO("Waiting on path data, sleeping for 1s...");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    
    // Get the entire shortest path to traverse
    Path pathObject = pathHandler.getPath();
    std::vector<Node> path = pathObject.getPath();
    
    Vector3f Xd;
    Vector3f X;
    int nodesVisited = 0;
    int totalNodes = pathObject.getTotalNodes();

    //ROS_INFO("totalNodes : %u", totalNodes);
    //ROS_INFO("Path valid? : %lu", path.size());

    while ( nodesVisited != totalNodes && ros::ok()) {
        ROS_INFO("Nodes Visited : %u", nodesVisited);

        int i = nodesVisited; // Easier right now - change later;

        ROS_INFO("END POINT : [%f %f]", (path[i+1]).x, (path[i+1]).y);
        ROS_INFO("START POINT : [%f %f]", path[i].x, path[i].y);

        Vector2f start_point(path[i].x, path[i].y);
        Vector2f end_point(path[i+1].x, path[i+1].y);
        float traj_angle = (float) atan2( end_point(1) - start_point(1), end_point(0) - start_point(0) );
        
        bool next_point = false;

        while (!next_point && ros::ok()) {
            float crosstrackError;

            Pose initPos = poseHandler.getPose(); // change initPose later in cleanup
            X(0) = initPos.x; X(1) = initPos.y; X(2) = initPos.yaw;

            ROS_INFO("[X, Y, YAW] : [%f, %f, %f]", X(0), X(1), X(2));
            Vector2f tempX(X(0), X(1));

            bool next_point = distanceToLineSegment(start_point, end_point, tempX, crosstrackError);
            ROS_INFO("CROSS TRACK ERROR : %f", crosstrackError);

            float headingError = traj_angle - X(2);
            ROS_INFO("HEADING ERROR : %f", headingError);
            
            vel.linear.x = velocity;
            vel.angular.z = -headingGain*headingError  - crosstrackGain*crosstrackError;
            
            velocity_publisher.publish(vel);
            
            loop_rate.sleep();
            ros::spinOnce();
        }

        ROS_INFO("-----------------------REACHED A NODE-----------------------");
        ROS_INFO("-----------------------REACHED A NODE-----------------------");
        ROS_INFO("-----------------------REACHED A NODE-----------------------");
        ROS_INFO("-----------------------REACHED A NODE-----------------------");
                
        nodesVisited++;
    }

    // while (ros::ok() && pathObject.getNodesVisited() != pathObject.totalNodes() ) {}
    
    return 0;
}
