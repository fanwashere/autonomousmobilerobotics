#include <ros/ros.h>
#include "pose.h"
#include "map.h"
#include "graph.h"
#include "visualizer.h"
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
    Visualizer visualizer(nodePublisher);

    // For control
    ros::Publisher controlPub = n.advertise<nav_msgs::Path>("/path", 100);

    // Wait for map to become available.
    while(ros::ok() && !mapHandler.hasData()) {
        ROS_INFO("Waiting on map data, sleeping for 500ms...");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    std::shared_ptr<Grid> grid = mapHandler.getGrid();
    grid->padObstacles(2);
    Graph graph(grid);
    visualizer.setScalingFactor(grid->getResolution());

    for (int i = 0; i < NUM_NODES; i++) {
        Coordinate coord;
        do {
            coord = grid->getRandomCoordinate();
        } while(grid->checkOccupancy(coord));

        std::shared_ptr<Node> newNode = std::make_shared<Node>(coord);
        graph.addNode(newNode);
        std::vector<std::shared_ptr<Edge>> edges = newNode->getEdges();
        ROS_INFO("Added node to graph [x: %d, y: %d] with %d edges", coord.getX(), coord.getY(), (int)edges.size());
        visualizer.drawNode(coord);
        visualizer.drawEdges(newNode);
    }

    /* Define waypoints */
    std::vector<Coordinate> waypoints;
    Coordinate waypoint1(1.2 * 1.0 / grid->getResolution(), 1.2 * 3.0 / grid->getResolution());
    Coordinate waypoint2(1.2 * 3.0 / grid->getResolution(), 1.2 * 3.5 / grid->getResolution());
    Coordinate waypoint3(1.2 * 4.5 / grid->getResolution(), 1.2 * 0.5 / grid->getResolution());

    waypoints.push_back(waypoint1);
    waypoints.push_back(waypoint2);
    waypoints.push_back(waypoint3);

    visualizer.drawWaypoint(waypoint1);
    visualizer.drawWaypoint(waypoint2);
    visualizer.drawWaypoint(waypoint3);

    Pose startingPose = poseHandler.getPose();
    Coordinate startingCoord(1.2 * startingPose.x / grid->getResolution(), 1.2 * startingPose.y / grid->getResolution());
    visualizer.drawWaypoint(startingCoord);

    std::vector<Coordinate> path = graph.findShortestPath(startingCoord, waypoints);
    visualizer.drawPath(path);

    // Converting Path<Coordinate> Path<nav_msgs::Path>
    nav_msgs::Path navPath;

    for (int i = 0; i < path.size(); ++i) {
        geometry_msgs::Point p;
        p.x = (float) (path[i].getX()) * grid->getResolution();
        p.y = (float) (path[i].getY()) * grid->getResolution();

        geometry_msgs::PoseStamped a;
        a.pose.position = p;
        navPath.poses.push_back(a);
    }

    ros::Rate rate(RATE);

    while(ros::ok()) {
        controlPub.publish(navPath);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
