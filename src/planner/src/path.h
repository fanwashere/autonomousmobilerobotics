#pragma once

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include "map.h"

class PathNode {
public:
    PathNode() = default;
    PathNode(float setX, float setY);
    
    void markAsVisited();
    bool isVisited() const;
    float x;
    float y;

private:
    bool visited;
};

class Path {
public:
    Path() = default;
    Path(nav_msgs::Path setPath);
    virtual ~Path() = default;

    void updateNodesVisited();
    int getNodesVisited() const;
    int getTotalNodes() const;
    std::vector<PathNode> getPath() const;
    
private:
    int totalNodes;
    int nodesVisited;
    std::vector<PathNode> path;
};

class PathHandler {
public:
    PathHandler() = default;
    virtual ~PathHandler() = default;

    bool hasPath() const;
    Path getPath() const;
    void callback(const nav_msgs::Path::ConstPtr &msg);

private:
    bool receivedPath = false;
    Path path;
};