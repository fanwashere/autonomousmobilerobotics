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
#include "visualizer.h"

using namespace Eigen;

namespace {
    const std::string NODE_NAME = "controller";
    const double RATE = 10.0;
    
    const float pi = 3.141592;

#ifdef LIVE
    const float headingGain = 0.8;
    const float crosstrackGain = 1.5;
    const float correctionX = 0.410;
    const float correctionY = 3.765;
#else
    const float headingGain = 0.25;
    const float crosstrackGain = 0.25;
    const float correctionX = 0.0;
    const float correctionY = 0.0;
#endif
    
    const float velocity = 0.1;
    const float delta_max = 35*pi/180;
    const float robot_length = 0.25;

    using PathCallback = boost::function<void(const nav_msgs::Path::ConstPtr&)>;
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
    
    float angleWrap(float angle) {
        return std::fmod(angle+pi, 2*pi) - pi;
    }

    void distanceToLineSegment(Vector2f p1, Vector2f p2, Vector2f x, float &crosstrackError) {
        Vector2f line_segment = p2-p1;
        Vector2f p1_to_x = x-p1;

        Vector2f projection = p1 + (line_segment.dot(p1_to_x) / line_segment.squaredNorm()) * line_segment;
        Vector2f distance = x - projection;
        
        float value = ((p2(0) - p1(0))*(x(1) - p1(1))) - ((x(0) - p1(0))*(p2(1) - p1(1)));

        if (value > 0) {
            crosstrackError = -1.0*distance.norm();
        } else {
            crosstrackError = distance.norm();
        }
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

    ros::Publisher visualizationPublisher = n.advertise<visualization_msgs::Marker>("robot", 1);
    Visualizer visualizer(visualizationPublisher);
    visualizer.setScalingFactor(1.0); // Since pose is given in actual meters, don't need scaling.

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
    std::vector<PathNode> path = pathObject.getPath();
    
    Vector3f Xd;
    Vector3f X;
    int nodesVisited = 0;
    int totalNodes = pathObject.getTotalNodes();

    while ( nodesVisited < totalNodes && ros::ok()) {
        ROS_INFO("Nodes Visited : %u", nodesVisited);

        int i = nodesVisited; // Easier right now - change later;
        
        Pose startPos = poseHandler.getPose(); // change initPose later in cleanup
        startPos.x += correctionX;
        startPos.y += correctionY;

        ROS_INFO("START POINT : [%f %f]", startPos.x, startPos.y);
        ROS_INFO("END POINT : [%f %f]", path[i].x, path[i].y);

        Vector2f start_point(startPos.x, startPos.y);
        Vector2f end_point(path[i].x, path[i].y);
        float traj_angle = (float) atan2( end_point(1) - start_point(1), end_point(0) - start_point(0) );
        
        while (ros::ok()) {
            Pose initPos = poseHandler.getPose(); // change initPose later in cleanup
            initPos.x += correctionX;
            initPos.y += correctionY;

            visualizer.drawRobot(initPos);

            X(0) = initPos.x; X(1) = initPos.y; X(2) = angleWrap(initPos.yaw);

            ROS_INFO("[X, Y, YAW] : [%f, %f, %f]", X(0), X(1), X(2));
            Vector2f tempX(X(0), X(1));

            if ( (end_point-tempX).norm() < 0.25) {
                ROS_INFO("Distance bw Destination and CurrPoint : %f", (end_point-tempX).norm());
                break;
            }

            float crosstrackError;
            distanceToLineSegment(start_point, end_point, tempX, crosstrackError);
            float headingError = std::max(-delta_max, std::min(delta_max, angleWrap(traj_angle - X(2))));
            
            ROS_INFO("CROSSTRACK ERROR : %f m", crosstrackError);
            ROS_INFO("HEADING ERROR : %f rad", headingError);
            
            vel.linear.x = velocity;
            vel.angular.z = (headingGain * headingError + crosstrackGain * crosstrackError);

            ROS_INFO("ANGULAR VEOCITY : %f", vel.angular.z);
            
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

    return 0;
}
