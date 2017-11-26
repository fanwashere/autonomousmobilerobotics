#include <ros/ros.h>
#include "path.h"

namespace {

}

Node::Node(float setX, float setY)
: x(setX)
, y(setY)
, visited(false)
{}

void Node::markAsVisited() {
    this->visited = true;
}

float Node::getX() const {
    return this->x;
}

float Node::getY() const {
    return this->y;
}

bool Node::isVisited() const {
    return this->visited;
}

void Path::updateNodesVisited() {
    nodesVisited++;
}

int Path::getNodesVisited() const{
    return nodesVisited;
}

int Path::getTotalNodes() const {
    return totalNodes;
}

Path::Path(nav_msgs::Path setPath) {
    ROS_INFO("GENERATING PATHS");

    auto nodes = setPath.poses;
    totalNodes = nodes.size();
    nodesVisited = 0;

    std::vector<Node> path;

    for (int i = 0 ; i < totalNodes; ++i) {
        auto cd = Node(nodes[i].pose.position.x, nodes[i].pose.position.y);
        path.push_back(cd);
    }
    
    ROS_INFO("SIZE OF COORDINATES : %lu", path.size());
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

