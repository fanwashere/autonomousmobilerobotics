#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "visualizer.h"
#include "graph.h"

namespace {
    const std::string VISUALIZATION_NAMESPACE = "graph";
    const std::string VISUALIZATION_FRAME_ID = "/map";
}

Visualizer::Visualizer(const ros::Publisher &setPublisher)
: publisher(setPublisher)
{}

void Visualizer::setScalingFactor(const double factor) {
    scalingFactor = factor;
}


void Visualizer::drawNode(const Coordinate &coord) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = VISUALIZATION_FRAME_ID;
    marker.header.stamp = ros::Time::now();
    marker.ns = VISUALIZATION_NAMESPACE;
    marker.id = iota++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = coord.getX() * scalingFactor;
    marker.pose.position.y = coord.getY() * scalingFactor;
    marker.pose.position.z = 0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0f;
    marker.color.a = 1.0f;

    publisher.publish(marker);
}

void Visualizer::drawWaypoint(const Coordinate &coord) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = VISUALIZATION_FRAME_ID;
    marker.header.stamp = ros::Time::now();
    marker.ns = VISUALIZATION_NAMESPACE;
    marker.id = iota++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.pose.position.x = coord.getX() * scalingFactor;
    marker.pose.position.y = coord.getY() * scalingFactor;
    marker.pose.position.z = 0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    publisher.publish(marker);
}

void Visualizer::drawEdge(const Coordinate &startCoord, const Coordinate &endCoord) {
    geometry_msgs::Point startPoint, endPoint;
    startPoint.x = startCoord.getX() * scalingFactor;
    startPoint.y = startCoord.getY() * scalingFactor;
    endPoint.x = endCoord.getX() * scalingFactor; 
    endPoint.y = endCoord.getY() * scalingFactor;

    visualization_msgs::Marker line;
    line.header.frame_id = VISUALIZATION_FRAME_ID;
    line.header.stamp = ros::Time::now();
    line.ns = VISUALIZATION_NAMESPACE;
    line.id = iota++;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = 0.05;
    line.color.r = 1.0f;
    line.color.a = 0.7f;

    line.points.push_back(startPoint);
    line.points.push_back(endPoint);

    publisher.publish(line);
}

void Visualizer::drawEdges(const std::shared_ptr<Node> &node) {
    std::vector<std::shared_ptr<Edge>> edges = node->getEdges();
    for (int i = 0; i < edges.size(); i++) {
        drawEdge(node->getCoordinate(), edges[i]->getDestination()->getCoordinate());
    }
}

void Visualizer::drawPath(const std::vector<Coordinate> &path) {
    visualization_msgs::Marker line;
    line.header.frame_id = VISUALIZATION_FRAME_ID;
    line.header.stamp = ros::Time::now();
    line.ns = VISUALIZATION_NAMESPACE;
    line.id = iota++;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.scale.x = 0.1;
    line.color.b = 1.0f;
    line.color.a = 1.0f;

    for (int i = 1; i < path.size(); i++) {
        geometry_msgs::Point startPoint;
        startPoint.x = path[i - 1].getX() * scalingFactor;
        startPoint.y = path[i - 1].getY() * scalingFactor;
        line.points.push_back(startPoint);

        geometry_msgs::Point endPoint;
        endPoint.x = path[i].getX() * scalingFactor;
        endPoint.y = path[i].getY() * scalingFactor;
        line.points.push_back(endPoint);
    }

    publisher.publish(line);
}
