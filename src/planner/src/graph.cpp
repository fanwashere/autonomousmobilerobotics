#include "graph.h"
#include "map.h"

namespace
{
    #define NEIGHBOR_MAX_DISTANCE 30.0
    #define Neighbor std::pair<std::shared_ptr<Node>, double>
}

Node::Node(const Coordinate &c) {
    coord = c;
}

void Node::addNeighbor(Node neighbor, double distance)
{
    neighbors.push_back(std::make_pair(std::make_shared<Node>(neighbor), distance));
}

std::vector<Neighbor> Node::getNeighbors() const
{
    return neighbors;
}

Coordinate Node::getCoordinate() const
{
    return coord;
}

void Node::assignId(int setId)
{
    id = setId;
}

Graph::Graph(const Grid &g)
{
    grid = g;
}

void Graph::addNode(Node &node)
{
    Coordinate coord = node.getCoordinate();
    node.assignId(iota++);

    int i;
    for (i = 0; i < nodes.size(); i++) {
        const Coordinate targetCoord = nodes[i]->getCoordinate();
	    const double distance = coord.distanceTo(targetCoord, grid.getResolution());
	    if (distance < NEIGHBOR_MAX_DISTANCE && grid.checkCollision(coord, targetCoord))
        {
            addVertex(node, *nodes[i], distance);
        }
    }

    nodes.push_back(std::make_shared<Node>(node));
}

void Graph::addVertex(Node n1, Node n2, double weight)
{
    n1.addNeighbor(n2, weight);
    n2.addNeighbor(n1, weight);
}
