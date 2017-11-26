#include <ros/ros.h>
#include "pose.h"
#include "map.h"
#include "graph.h"
#include <geometry_msgs/Point.h>
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

    /* Test Dijkstra */
    Coordinate start(5, 5);
    visualization_msgs::Marker startMarker;
    startMarker.header.frame_id = "/map";
    startMarker.header.stamp = ros::Time::now();
    startMarker.ns = "nodes";
    startMarker.id = 999999;
    startMarker.type = visualization_msgs::Marker::SPHERE;
    startMarker.pose.position.x = start.getX() * grid->getResolution();
    startMarker.pose.position.y = start.getY() * grid->getResolution();
    startMarker.pose.position.z = 0;
    startMarker.scale.x = 0.3;
    startMarker.scale.y = 0.3;
    startMarker.scale.z = 0.3;
    startMarker.color.b = 1.0f;
    startMarker.color.a = 1.0f;
    nodePublisher.publish(startMarker);

    Coordinate end(95, 95);
    visualization_msgs::Marker endMarker;
    endMarker.header.frame_id = "/map";
    endMarker.header.stamp = ros::Time::now();
    endMarker.ns = "nodes";
    endMarker.id = 9999999;
    endMarker.type = visualization_msgs::Marker::SPHERE;
    endMarker.pose.position.x = end.getX() * grid->getResolution();
    endMarker.pose.position.y = end.getY() * grid->getResolution();
    endMarker.pose.position.z = 0;
    endMarker.scale.x = 0.3;
    endMarker.scale.y = 0.3;
    endMarker.scale.z = 0.3;
    endMarker.color.b = 1.0f;
    endMarker.color.a = 1.0f;
    nodePublisher.publish(endMarker);

    std::vector<Coordinate> path = graph.findShortestPath(start, end);

    visualization_msgs::Marker pathLine;
    pathLine.header.frame_id = "/map";
    pathLine.header.stamp = ros::Time::now();
    pathLine.ns = "nodes";
    pathLine.id = 12345123;
    pathLine.type = visualization_msgs::Marker::LINE_LIST;
    pathLine.scale.x = 0.1;
    pathLine.color.b = 1.0f;
    pathLine.color.a = 1.0f;
    for (int i = 1; i < path.size(); i++) {
        geometry_msgs::Point p0;
        p0.x = path[i - 1].getX() * grid->getResolution();
        p0.y = path[i - 1].getY() * grid->getResolution();
        pathLine.points.push_back(p0);

        geometry_msgs::Point p1;
        p1.x = path[i].getX() * grid->getResolution();
        p1.y = path[i].getY() * grid->getResolution();
        pathLine.points.push_back(p1);
    }
    nodePublisher.publish(pathLine);

    ros::Rate rate(RATE);

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
