#pragma once

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include "map.h"

class Node {
public:
    Node() = default;
    Node(float setX, float setY);
    
    void markAsVisited();
    bool isVisited() const;
    float getX() const;
    float getY() const;

private:
    float x;
    float y;
    bool visited;
};

class Path {
public:
    Path() = default;
    Path(nav_msgs::Path setPath);
    virtual ~Path() = default;

private:
    int totalNodes;
    int nodesVisited;
    std::vector<Node> path;
};

class PathHandler {
public:
    PathHandler() = default;
    virtual ~PathHandler() = default;

    bool hasPath() const;
    std::shared_ptr<Path> getPath() const;
    void callback(const nav_msgs::Path::ConstPtr &msg);

private:
    bool receivedPath = false;
    std::shared_ptr<Path> path;
};