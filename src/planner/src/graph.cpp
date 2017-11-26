#include <ros/ros.h>
#include "map.h"
#include "graph.h"
#include <queue>

namespace {
    const double NEIGHBOR_MAX_DISTANCE = 2.0;
    const uint32_t INF = 0x3f3f3f3f;
    struct Compare {
        constexpr bool operator()(std::pair<std::shared_ptr<Node>, double> const &n1,
                                  std::pair<std::shared_ptr<Node>, double> const &n2) const noexcept
        { return n1.second > n2.second; }
    };
}

Node::Node(const Coordinate &c)
: id(-1)
, coord(c)
{}

void Node::addEdge(std::shared_ptr<Node> to, double weight) {
    std::shared_ptr<Edge> edgePtr = std::make_shared<Edge>(to, weight);
    edges.push_back(edgePtr);
}

std::vector<std::shared_ptr<Edge>> Node::getEdges() const {
    return edges;
}

Coordinate Node::getCoordinate() const {
    return coord;
}

void Node::assignId(int setId) {
    id = setId;
}

int Node::getId() const {
    return id;
}

Edge::Edge(std::shared_ptr<Node> setDestination, double setWeight)
: destination(setDestination)
, weight(setWeight)
{}

std::shared_ptr<Node> Edge::getDestination() const {
    return destination;
}

double Edge::getWeight() const {
    return weight;
}

Graph::Graph(std::shared_ptr<Grid> g)
: iota(0)
, grid(g)
{}

void Graph::addNode(std::shared_ptr<Node> node) {
    Coordinate coord = node->getCoordinate();
    node->assignId(iota++);

    for (int i = 0; i < nodes.size(); i++) {
        const Coordinate targetCoord = nodes[i]->getCoordinate();
        const double distance = coord.distanceTo(targetCoord, grid->getResolution());

        if (distance < NEIGHBOR_MAX_DISTANCE && !grid->checkCollision(coord, targetCoord)) {
            addVertex(node, nodes[i], distance);
        }
    }

    nodes.push_back(node);
}

void Graph::addVertex(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double weight) {
    n1->addEdge(n2, weight);
    n2->addEdge(n1, weight);
}

std::vector<Coordinate> Graph::findShortestPath(const Coordinate &start, const Coordinate &end) {
    std::vector<std::shared_ptr<Node>> prev(nodes.size() + 2); // +2 for start and end node, intentionally init as null
    std::vector<double> dist(nodes.size(), INF);
    std::vector<bool> inQueue(nodes.size() + 2, false);

    // Add starting node
    addNode(std::make_shared<Node>(start));
    dist.push_back(0); // Init distance 0
    std::shared_ptr<Node> startNodePtr = nodes.back();
    ROS_INFO("[Dijkstra] Inserted start node.");

    // Add ending node
    addNode(std::make_shared<Node>(end));
    dist.push_back(INF); // Init distance unknown
    std::shared_ptr<Node> endNodePtr = nodes.back();
    ROS_INFO("[Dijkstra] Inserted end node.");

    // Create priority queue
    auto cmp = [](std::pair<std::shared_ptr<Node>, double> n1, std::pair<std::shared_ptr<Node>, double> n2) { ROS_INFO("Cmp %f %f choosing %d", n1.second, n2.second, n1.second < n2.second ? 1 : 2); return n1.second < n2.second; };
    std::priority_queue<std::pair<std::shared_ptr<Node>, double>, std::vector<std::pair<std::shared_ptr<Node>, double>>, Compare> pq;
    ROS_INFO("[Dijkstra] Created priority queue.");

    pq.push(std::pair<std::shared_ptr<Node>, double>(startNodePtr, 0.0));
    inQueue[startNodePtr->getId()] = true;
    ROS_INFO("[Dijkstra] Inserted start node into priority queue, now size %d.", (int)pq.size());

    while (!pq.empty())
    {
        std::pair<std::shared_ptr<Node>, double> uPair = pq.top();
        std::shared_ptr<Node> u = uPair.first;
        double distance = uPair.second;
        pq.pop();
        inQueue[u->getId()] = false;

	ROS_INFO("[Dijkstra] Dequeued one item from priority queue, now size %d.", (int)pq.size());

        if (u->getId() == endNodePtr->getId())
        {
            ROS_INFO("[Dijkstra] End node found, exiting algorithm.");
            //break;
        }

        std::vector<std::shared_ptr<Edge>> connections = u->getEdges();
        int enqueuedCount = 0;
        for (int j = 0; j < connections.size(); j++)
        {
            std::shared_ptr<Node> v = connections[j]->getDestination();
            double totalDistance = distance + connections[j]->getWeight();
            if (totalDistance < dist[v->getId()])
            {
                dist[v->getId()] = totalDistance;
                prev[v->getId()] = u;
                if (!inQueue[v->getId()]) {
                    pq.push(std::pair<std::shared_ptr<Node>, double>(v, totalDistance));
                    enqueuedCount++;
                    inQueue[v->getId()] = true;
                }
            }
        }
        ROS_INFO("[Dijkstra] For current node, %d connections found, %d re-enqueued.", (int)connections.size(), enqueuedCount);
    }

    // Construct path
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> u = endNodePtr;
    ROS_INFO("[Dijkstra] Starting path reconstruction with end node ID %d", u->getId());
    path.push_back(u);
    while (prev[u->getId()] != nullptr) {
        path.push_back(prev[u->getId()]);
        u = prev[u->getId()];
    }
    std::reverse(path.begin(), path.end());

    return nodesToCoordinates(path);
}

std::vector<Coordinate> Graph::findShortestPath(const Coordinate &start, const std::vector<Coordinate> &waypoints) {
    std::vector<Coordinate> path;

    for (int i = 0; i < waypoints.size(); i++)
    {
        const Coordinate subpathStart = i == 0 ? start : waypoints[i - 1];
        const Coordinate subpathEnd = waypoints[i];
        std::vector<Coordinate> subpath = findShortestPath(subpathStart, subpathEnd);
        path.insert(path.end(), subpath.begin(), subpath.end());
    }
}

std::vector<Coordinate> Graph::nodesToCoordinates(std::vector<std::shared_ptr<Node>> nodeVector) const {
    std::vector<Coordinate> coords;

    for (int i = 0; i < nodeVector.size(); i++)
    {
        coords.push_back(nodeVector[i]->getCoordinate());
    }

    return coords;
}
