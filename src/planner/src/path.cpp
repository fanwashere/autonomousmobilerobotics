#include <ros/ros.h>
#include "path.h"

PathNode::PathNode(float setX, float setY)
: x(setX)
, y(setY)
, visited(false)
{}

void PathNode::markAsVisited() {
    this->visited = true;
}

bool PathNode::isVisited() const {
    return this->visited;
}

void Path::updateNodesVisited() {
    nodesVisited++;
}

int Path::getNodesVisited() const{
    return nodesVisited;
}

int Path::getTotalNodes() const {
    return this->totalNodes;
}

std::vector<PathNode> Path::getPath() const {
    return this->path;
}

Path::Path(nav_msgs::Path setPath) {
    ROS_INFO("GENERATING PATHS");

    auto nodes = setPath.poses;
    totalNodes = nodes.size();
    nodesVisited = 0;

    for (int i = 0 ; i < totalNodes; ++i) {
        auto cd = PathNode(nodes[i].pose.position.x, nodes[i].pose.position.y);
        path.push_back(cd);
    }
}

bool PathHandler::hasPath() const {
    return receivedPath;
}

Path PathHandler::getPath() const {
    return path;
}

void PathHandler::callback(const nav_msgs::Path::ConstPtr &msg) {
    if (!receivedPath) {
        receivedPath = true;
        path = Path(*msg);
    }
}

