#include <ros/ros.h>
#include "pose.h"
#include "map.h"
#include "graph.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

namespace {
    const std::string NODE_NAME = "planner";
    const double RATE = 1.0;
    const int NUM_NODES = 200;

    using MapCallback = boost::function<void(const nav_msgs::OccupancyGrid::ConstPtr&)>;
    using PoseSimCallback = boost::function<void(const gazebo_msgs::ModelStates::ConstPtr&)>;
    using PoseLiveCallback = boost::function<void(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    MapHandler mapHandler;

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

    MapCallback mapCallback = boost::bind(&MapHandler::callback, &mapHandler, _1);
    auto mapSubscriber = n.subscribe("/map", 1, mapCallback);
    ROS_INFO("Subscribed to /map topic");

    // For visualization
    ros::Publisher nodePublisher = n.advertise<visualization_msgs::Marker>("nodes", 500);

    // Wait for map to become available.
    while(ros::ok() && !mapHandler.hasData()) {
        ROS_INFO("Waiting on map data, sleeping for 500ms...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    std::shared_ptr<Grid> grid = mapHandler.getGrid();
    grid->padObstacles(2);
    Graph graph(grid);

    for (int i = 0; i < NUM_NODES; i++) {
        Coordinate coord;
        do {
            coord = grid->getRandomCoordinate();
        } while(grid->checkOccupancy(coord));

        std::shared_ptr<Node> newNode = std::make_shared<Node>(coord);

        graph.addNode(newNode);

        std::vector<std::shared_ptr<Edge>> edges = newNode->getEdges();
        // ROS_INFO("Added node to graph [x: %d, y: %d] with %d edges", coord.getX(), coord.getY(), (int)edges.size());
    }

    // WRITE A FUNCTION TO GO FROM COORDINATE PATH -> NAV MSGS PATH

    // Control node publisher
    ROS_INFO("ADVERTISING TO path");
    ros::Publisher controlPub = n.advertise<nav_msgs::Path>("path", 10);
    nav_msgs::Path path;

    // -- This section is for testing the robot -- /
    geometry_msgs::Point p1, p2, p3;

    p1.x = 0; p1.y = 0;
    p2.x = 3; p2.y = 4;
    p3.x = 8; p3.y = 2;

    geometry_msgs::PoseStamped a1, a2, a3;

    a1.pose.position = p1;
    a2.pose.position = p2;
    a3.pose.position = p3;

    // geometry_msgs::PoseStamped p [] = {
    //     a1, a2, a3    
    // };

    std::vector<geometry_msgs::PoseStamped> p;
    p.push_back(a1);
    p.push_back(a2);
    p.push_back(a3);

    path.poses = p;

    controlPub.publish(path);

    // -- End of Test Section -- /
    
    // std::vector<geometry_msgs::Point> path;
    //Coordinate start(0, 0);
    //Coordinate end(10, 10);
    //std::vector<Coordinate> path = graph.findShortestPath(start, end);

    /*
    Pose pose = poseHandler.getPose();
    Coordinate initCoord(pose.x, pose.y);
    Node initNode(initCoord);
    */

    ros::Rate rate(RATE);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
