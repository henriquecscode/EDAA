#include "node.h"
#include <vector>
#include <set>
#include <limits>

using namespace std;

template <typename NodeT, typename EdgeT>
Node<NodeT, EdgeT>::Node(NodeT data)
{
    this->data = data;
    this->resetNode();
}

template <typename NodeT, typename EdgeT>
NodeT &Node<NodeT, EdgeT>::getData()
{
    return data;
}

template <typename NodeT, typename EdgeT>
bool Node<NodeT, EdgeT>::addIncomingEdge(Edge<NodeT, EdgeT> *edge)
{
    incomingEdges.push_back(edge);
    if (incomingEdgesByNode.find(edge->getSource()) == incomingEdgesByNode.end())
    {
        incomingEdgesByNode[edge->getSource()] = vector<Edge<NodeT, EdgeT> *>();
    }
    incomingEdgesByNode[edge->getSource()].push_back(edge);
    return true;
}

template <typename NodeT, typename EdgeT>
bool Node<NodeT, EdgeT>::addOutgoingEdge(Edge<NodeT, EdgeT> *edge)
{
    outgoingEdges.push_back(edge);
    if (outgoingEdgesByNode.find(edge->getDest()) == outgoingEdgesByNode.end())
    {
        outgoingEdgesByNode[edge->getDest()] = vector<Edge<NodeT, EdgeT> *>();
    }
    outgoingEdgesByNode[edge->getDest()].push_back(edge);
    return true;
}

template <typename NodeT, typename EdgeT>
bool Node<NodeT, EdgeT>::addEdge(Edge<NodeT, EdgeT> *edge)
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
        throw "Edge<NodeT, EdgeT> does not connect to this node";
    }
    return true;
}
template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::resetNode()
{
    found = false;
    distance = numeric_limits<double>::infinity();
    previousEdge = nullptr;
    auxIncomingEdges.clear();
    auxOutgoingEdges.clear();
}
template <typename NodeT, typename EdgeT>
bool Node<NodeT, EdgeT>::isFound()
{
    return found;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::find()
{
    found = true;
}

template <typename NodeT, typename EdgeT>
double Node<NodeT, EdgeT>::getNodeDistance()
{
    return distance;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::setNodeDistance(double distance)
{
    this->distance = distance;
}

template <typename NodeT, typename EdgeT>
Edge<NodeT, EdgeT> *Node<NodeT, EdgeT>::getPreviousEdge()
{
    return previousEdge;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::setPreviousEdge(Edge<NodeT, EdgeT> *edge)
{
    this->previousEdge = edge;
}

template <typename NodeT, typename EdgeT>
set<Edge<NodeT, EdgeT> *> Node<NodeT, EdgeT>::getAuxIncomingEdges()
{
    return auxIncomingEdges;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::addAuxIncomingEdge(Edge<NodeT, EdgeT> *edge)
{
    this->auxIncomingEdges.add(edge);
}

template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::removeAuxIncomingEdge(Edge<NodeT, EdgeT> *edge)
{
    this->auxIncomingEdges.remove(edge);
}

template <typename NodeT, typename EdgeT>
set<Edge<NodeT, EdgeT> *> Node<NodeT, EdgeT>::getAuxOutgoingEdges()
{
    return auxOutgoingEdges;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::addAuxOutgoingEdge(Edge<NodeT, EdgeT> *edge)
{
    this->auxOutgoingEdges.add(edge);
}

template <typename NodeT, typename EdgeT>
void Node<NodeT, EdgeT>::removeAuxOutgoingEdge(Edge<NodeT, EdgeT> *edge)
{
    this->auxOutgoingEdges.remove(edge);
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Node<NodeT, EdgeT>::getIncomingEdges()
{
    return incomingEdges;
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Node<NodeT, EdgeT>::getOutgoingEdges()
{
    return outgoingEdges;
}

template <typename NodeT, typename EdgeT>
map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> Node<NodeT, EdgeT>::getIncomingEdgesByNode()
{
    return incomingEdgesByNode;
}

template <typename NodeT, typename EdgeT>
map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> Node<NodeT, EdgeT>::getOutgoingEdgesByNode()
{
    return outgoingEdgesByNode;
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Node<NodeT, EdgeT>::getIncomingEdgesFromNode(Node<NodeT, EdgeT> *node)
{
    return incomingEdgesByNode[node];
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Node<NodeT, EdgeT>::getOutgoingEdgesToNode(Node<NodeT, EdgeT> *node)
{
    return outgoingEdgesByNode[node];
}