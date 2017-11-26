#include <ros/ros.h>
#include "path.h"

namespace {

}

Node::Node(geometry_msgs::Point setCoordinates)
: x(setCoordinates.x)
, y(setCoordinates.y)
, visited(false)
{}

void Node::markAsVisited() {
    this->visited = true;
}

Path::Path(nav_msgs::Path setPath) {
    ROS_INFO("GENERATING PATHS");

    auto nodes = setPath.poses;
    totalNodes = nodes.size();
    nodesVisited = 0;

    std::vector<Node> path;

    for (int i = 0 ; i < totalNodes; ++i) {
        ROS_INFO("Coordinates [%f, %f]", nodes[i].pose.position.x, nodes[i].pose.position.y);

        // path.push_back(Node(nodes[i].pose.position));
    }
}

bool PathHandler::hasPath() const {
    return receivedPath;
}

std::shared_ptr<Path> PathHandler::getPath() const {
    return path;
}

void PathHandler::callback(const nav_msgs::Path::ConstPtr &msg) {
    if (!receivedPath) {
        receivedPath = true;
        path = std::make_shared<Path>(*msg);
    }
}

