#include <ros/ros.h>
#include "pose.h"
#include "map.h"
#include "graph.h"
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

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
        ROS_INFO("Added node to graph [x: %d, y: %d] with %d edges", coord.getX(), coord.getY(), (int)edges.size());

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "nodes";
        marker.id = newNode->getId();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = coord.getX() * grid->getResolution();
        marker.pose.position.y = coord.getY() * grid->getResolution();
        marker.pose.position.z = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 1.0f;
        marker.color.a = 1.0f;

        nodePublisher.publish(marker);

        for (int j = 0; j < edges.size(); j++) {
            std::shared_ptr<Edge> edge = edges[j];
            std::shared_ptr<Node> destinationNode = edge->getDestination();

            visualization_msgs::Marker edgeLine;
            edgeLine.header.frame_id = "/map";
            edgeLine.header.stamp = ros::Time::now();
            edgeLine.ns = "nodes";
            edgeLine.id = newNode->getId() * NUM_NODES + destinationNode->getId();
            edgeLine.type = visualization_msgs::Marker::LINE_STRIP;
            edgeLine.scale.x = 0.05;
            edgeLine.color.r = 1.0f;
            edgeLine.color.a = 0.7f;

            geometry_msgs::Point startPoint, endPoint;
            startPoint.x = coord.getX() * grid->getResolution();
            startPoint.y = coord.getY() * grid->getResolution();
            endPoint.x = destinationNode->getCoordinate().getX() * grid->getResolution();
            endPoint.y = destinationNode->getCoordinate().getY() * grid->getResolution();

            edgeLine.points.push_back(startPoint);
            edgeLine.points.push_back(endPoint);

            nodePublisher.publish(edgeLine);
        }
    }

    // WRITE A FUNCTION TO GO FROM COORDINATE PATH -> NAV MSGS PATH

    // Control node publisher
    ros::Publisher controlPub = n.advertise<nav_msgs::Path>("path", 100);
    
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
