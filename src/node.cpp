#include "node.h"
#include <vector>

using namespace std;

template <class T>
Node<T>::Node(T data)
{
    this->data = data;
    this->resetNode();
}

template <class T>
T &Node<T>::getData()
{
    return data;
}

template <class T>
bool Node<T>::addIncomingEdge(Edge *edge)
{
    incomingEdges.push_back(edge);
    if (incomingEdgesByNode.find(edge->getSource()) == incomingEdgesByNode.end())
    {
        incomingEdgesByNode[edge->getSource()] = vector<Edge *>();
    }
    incomingEdgesByNode[edge->getSource()].push_back(edge);
    return true;
}

template <class T>
bool Node<T>::addOutgoingEdge(Edge *edge)
{
    outgoingEdges.push_back(edge);
    if (outgoingEdgesByNode.find(edge->getDest()) == outgoingEdgesByNode.end())
    {
        outgoingEdgesByNode[edge->getDest()] = vector<Edge *>();
    }
    outgoingEdgesByNode[edge->getDest()].push_back(edge);
    return true;
}

template <class T>
bool Node<T>::addEdge(Edge *edge)
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
template <class T>
void Node<T>::resetNode()
{
    distance = numeric_limits<double>::infinity();
    previousEdge = nullptr;
}

template <class T>
double Node<T>::getNodeDistance()
{
    return distance;
}

template <class T>
void Node<T>::setNodeDistance(double distance)
{
    this->distance = distance;
}

template <class T>
Edge *Node<T>::getPreviousEdge()
{
    return previousEdge;
}

template <class T>
void Node<T>::setPreviousEdge(Edge *edge)
{
    this->previousEdge = edge;
    return edge;
}

template <class T>
vector<Edge *> Node<T>::getIncomingEdges()
{
    return incomingEdges;
}

template <class T>
vector<Edge *> Node<T>::getOutgoingEdges()
{
    return outgoingEdges;
}

template <class T>
map<Node<T> *, vector<Edge *>> Node<T>::getIncomingEdgesByNode()
{
    return incomingEdgesByNode;
}

template <class T>
map<Node<T> *, vector<Edge *>> Node<T>::getOutgoingEdgesByNode()
{
    return outgoingEdgesByNode;
}

template <class T>
vector<Edge *> Node<T>::getIncomingEdgesFromNode(Node<T> *node)
{
    return incomingEdgesByNode[node];
}

template <class T>
vector<Edge *> Node<T>::getOutgoingEdgesToNode(Node<T> *node)
{
    return outgoingEdgesByNode[node];
}