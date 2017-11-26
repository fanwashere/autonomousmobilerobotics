#pragma once

#include <ros/ros.h>
#include "map.h"

class Visualizer {
public:
    Visualizer(const ros::Publisher &setPublisher);
    void setScalingFactor(const double factor);
    void drawNode(const Coordinate &coord);
    void drawWaypoint(const Coordinate &coord);
    void drawEdge(const Coordinate &startCoord, const Coordinate &endCoord);
    void drawPath(const std::vector<Coordinate> &path);

private:
    ros::Publisher publisher;
    double scalingFactor;
    int iota = 0;
};
