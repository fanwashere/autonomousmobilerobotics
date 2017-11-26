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
    auto nodes = setPath.poses;
    totalNodes = nodes.size();
    nodesVisited = 0;

    std::vector<Node> path;

    for (int i = 0 ; i < totalNodes; ++i) {
        path.push_back(Node(nodes[i].pose.position));
    }
}


