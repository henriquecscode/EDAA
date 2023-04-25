#include "node.h"
#include "airport.h"
#include "edge.h"
#include <vector>
#include <set>
#include <limits>
#include <algorithm>

using namespace std;


Node::Node(Airport data) : data(data)
{
    this->resetNode();
}


Airport &Node::getData()
{
    return data;
}


bool Node::addIncomingEdge(Edge *edge)
{
    incomingEdges.push_back(edge);
    if (incomingEdgesByNode.find(edge->getSource()) == incomingEdgesByNode.end())
    {
        incomingEdgesByNode[edge->getSource()] = vector<Edge *>();
    }
    incomingEdgesByNode[edge->getSource()].push_back(edge);
    return true;
}


bool Node::addOutgoingEdge(Edge *edge)
{
    outgoingEdges.push_back(edge);
    if (outgoingEdgesByNode.find(edge->getDest()) == outgoingEdgesByNode.end())
    {
        outgoingEdgesByNode[edge->getDest()] = vector<Edge *>();
    }
    outgoingEdgesByNode[edge->getDest()].push_back(edge);
    return true;
}


bool Node::addEdge(Edge *edge)
{
    if (edge->getSource() == this)
    {
        addOutgoingEdge(edge);
        edge->getDest()->addIncomingEdge(edge);
    }
    else if (edge->getDest() == this)
    {
        edge->getSource()
            ->addOutgoingEdge(edge);
        addIncomingEdge(edge);
    }
    else
    {
        throw "Edge does not connect to this node";
    }
    return true;
}

void Node::resetNode()
{
    found = false;
    distance = numeric_limits<double>::infinity();
    previousEdge = nullptr;
    auxIncomingEdges.clear();
    auxOutgoingEdges.clear();
}

bool Node::isFound()
{
    return found;
}


void Node::find()
{
    found = true;
}


double Node::getNodeDistance()
{
    return distance;
}


void Node::setNodeDistance(double distance)
{
    this->distance = distance;
}


Edge *Node::getPreviousEdge()
{
    return previousEdge;
}


void Node::setPreviousEdge(Edge *edge)
{
    this->previousEdge = edge;
}


set<Edge *> Node::getAuxIncomingEdges()
{
    return auxIncomingEdges;
}


void Node::addAuxIncomingEdge(Edge *edge)
{
    this->auxIncomingEdges.insert(edge);
}


void Node::removeAuxIncomingEdge(Edge *edge)
{
    this->auxIncomingEdges.erase(edge);
}


set<Edge *> Node::getAuxOutgoingEdges()
{
    return auxOutgoingEdges;
}


void Node::addAuxOutgoingEdge(Edge *edge)
{
    this->auxOutgoingEdges.insert(edge);
}


void Node::removeAuxOutgoingEdge(Edge *edge)
{
    this->auxOutgoingEdges.erase(edge);
}


vector<Edge *> Node::getIncomingEdges()
{
    return incomingEdges;
}


vector<Edge *> Node::getOutgoingEdges()
{
    return outgoingEdges;
}


map<Node *, vector<Edge *>> Node::getIncomingEdgesByNode()
{
    return incomingEdgesByNode;
}


map<Node *, vector<Edge *>> Node::getOutgoingEdgesByNode()
{
    return outgoingEdgesByNode;
}


vector<Edge *> Node::getIncomingEdgesFromNode(Node *node)
{
    return incomingEdgesByNode[node];
}


vector<Edge *> Node::getOutgoingEdgesToNode(Node *node)
{
    return outgoingEdgesByNode[node];
}