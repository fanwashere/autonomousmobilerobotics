#include "graph.h"
#include "map.h"
#include <queue>

namespace
{
    #define NEIGHBOR_MAX_DISTANCE 30.0
    #define INF 0x3f3f3f3f
    #define Neighbor std::pair<std::shared_ptr<Node>, double>
}

Node::Node(const Coordinate &c) {
    coord = c;
}

void Node::addEdge(std::shared_ptr<Node> to, double weight)
{
    Edge edge(to, weight);
    std::shared_ptr<Edge> edgePtr = std::make_shared<Edge>(edge); // Might not need shared_ptr
    edges.push_back(edgePtr);
}

std::vector<std::shared_ptr<Edge>> Node::getEdges() const
{
    return edges;
}

Coordinate Node::getCoordinate() const
{
    return coord;
}

void Node::assignId(int setId)
{
    id = setId;
}

int Node::getId() const
{
    return id;
}

Edge::Edge(std::shared_ptr<Node> setDestination, double setWeight)
{
    destination = setDestination;
    weight = setWeight;
}

Graph::Graph(const Grid &g)
{
    grid = g;
}

void Graph::addNode(Node &node)
{
    Coordinate coord = node.getCoordinate();
    node.assignId(iota++);
    std::shared_ptr<Node> nodePtr = std::make_shared<Node>(node);

    int i;
    for (i = 0; i < nodes.size(); i++) {
        const Coordinate targetCoord = nodes[i]->getCoordinate();
	const double distance = coord.distanceTo(targetCoord, grid.getResolution());
	if (distance < NEIGHBOR_MAX_DISTANCE && grid.checkCollision(coord, targetCoord))
        {
            addVertex(nodePtr, nodes[i], distance);
        }
    }

    nodes.push_back(nodePtr);
}

void Graph::addVertex(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, double weight)
{
    n1->addEdge(n2, weight);
    n2->addEdge(n1, weight);
}

std::vector<Coordinate> Graph::findShortestPath(const Coordinate &start, const Coordinate &end)
{
    std::vector<std::shared_ptr<Node>> prev(nodes.size() + 2); // +2 for start and end node, intentionally init as null
    std::vector<double> dist(nodes.size(), INF);

    // Add starting node
    Node startNode(start);
    addNode(startNode);
    dist.push_back(0); // Init distance 0
    std::shared_ptr<Node> startNodePtr = nodes.back();

    // Add ending node
    Node endNode(end);
    addNode(endNode);
    dist.push_back(INF); // Init distance unknown
    std::shared_ptr<Node> endNodePtr = nodes.back();

    // Create priority queue
    auto cmp = [](std::pair<std::shared_ptr<Node>, double> n1, std::pair<std::shared_ptr<Node>, double> n2) { return n1.second < n2.second; };
    std::priority_queue<std::pair<std::shared_ptr<Node>, double>, std::vector<std::pair<std::shared_ptr<Node>, double>>, decltype(cmp)> pq(cmp);

    pq.push(std::pair<std::shared_ptr<Node>, double>(startNodePtr, 0.0));

    while (!pq.empty())
    {
        std::pair<std::shared_ptr<Node>, double> uPair = pq.top();
        std::shared_ptr<Node> u = uPair.first;
        double distance = uPair.second;
        pq.pop();

        if (u->getId() == endNodePtr->getId())
        {
            break;
        }

        std::vector<std::shared_ptr<Edge>> connections = u->getEdges();
        int j;
        for (j = 0; j < connections.size(); j++)
        {
            std::shared_ptr<Node> v = connections[j]->destination;
            double totalDistance = distance + connections[j]->weight;
            if (totalDistance < dist[v->getId()])
            {
                dist[v->getId()] = totalDistance;
                prev[v->getId()] = u;
                pq.push(std::pair<std::shared_ptr<Node>, double>(v, totalDistance));
            }
        }
    }

    // Construct path
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> u = endNodePtr;
    while (prev[u->getId()] != nullptr) {
        path.push_back(prev[u->getId()]);
        u = prev[u->getId()];
    }
    std::reverse(path.begin(), path.end());

    return nodesToCoordinates(path);
}

std::vector<Coordinate> Graph::findShortestPath(const Coordinate &start, const std::vector<Coordinate> &waypoints)
{
    std::vector<Coordinate> path;

    int i;
    for (i = 0; i < waypoints.size(); i++)
    {
        const Coordinate subpathStart = i == 0 ? start : waypoints[i - 1];
        const Coordinate subpathEnd = waypoints[i];
        std::vector<Coordinate> subpath = findShortestPath(subpathStart, subpathEnd);
        path.insert(path.end(), subpath.begin(), subpath.end());
    }
}

std::vector<Coordinate> Graph::nodesToCoordinates(std::vector<std::shared_ptr<Node>> nodeVector) const
{
    std::vector<Coordinate> coords;
    int i;
    for (i = 0; i < nodeVector.size(); i++)
    {
        coords.push_back(nodeVector[i]->getCoordinate());
    }

    return coords;
}
