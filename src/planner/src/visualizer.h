#pragma once

#include <ros/ros.h>
#include "map.h"
#include "graph.h"
#include "pose.h"

class Visualizer {
public:
    Visualizer(const ros::Publisher &setPublisher);
    void setScalingFactor(const double factor);
    void drawNode(const Coordinate &coord);
    void drawWaypoint(const Coordinate &coord);
    void drawEdge(const Coordinate &startCoord, const Coordinate &endCoord);
    void drawEdges(const std::shared_ptr<Node> &node);
    void drawPath(const std::vector<Coordinate> &path);
    void drawRobot(const Pose &pose);

private:
    ros::Publisher publisher;
    double scalingFactor;
    int iota = 10;
};
