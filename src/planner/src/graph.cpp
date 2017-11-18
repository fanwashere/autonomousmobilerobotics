#include "graph.h"

void Node::getCoordinate()
{
    return coord;
}

void Node::assignId(int setId)
{
    id = setId;
}

void Node::addNeighbor(Node neighbor, double distance) {
    neighbors.push_back(std::make_pair(Node, distance));
}

std::vector<Node> Node::getNeighbors()
{
    std::vector<Node> neighborNodes;

    int i;
    for (i = 0; i < neighbors.size(); i++) {
        neighborNodes.push_back(neighbors[i].first);
    }

    return neighborNodes;
}

void Graph::addNode(Node node)
{
    const Coordinate coord = node.getCoordinate();
    node.assignId(iota++);

    int i;
    for (i = 0; i < nodes.size(); i++) {
        const Coordinate targetCoord = nodes[i].getCoordinate();
        if (coord.distanceTo(targetCoord) < 30 && grid.checkCollision(coord, targetCoord)) // TODO set constant
        {
            addVertex(node, nodes[i]);
        }
    }

    nodes.push_back(node);
}

void Graph::addVertex(Node n1, Node n2, double weight)
{
    n1.addNeighbor(n2, weight);
    n2.addNeighbor(n1, weight);
}