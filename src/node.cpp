#include "node.h"
#include <vector>
#include <set>

using namespace std;

template <typename NodeT>
Node<NodeT>::Node(NodeT data)
{
    this->data = data;
    this->resetNode();
}

template <typename NodeT>
NodeT &Node<NodeT>::getData()
{
    return data;
}

template <typename NodeT>
bool Node<NodeT>::addIncomingEdge(Edge<EdgeT> *edge)
{
    incomingEdges.push_back(edge);
    if (incomingEdgesByNode.find(edge->getSource()) == incomingEdgesByNode.end())
    {
        incomingEdgesByNode[edge->getSource()] = vector<Edge<EdgeT> *>();
    }
    incomingEdgesByNode[edge->getSource()].push_back(edge);
    return true;
}

template <typename NodeT>
bool Node<NodeT>::addOutgoingEdge(Edge<EdgeT> *edge)
{
    outgoingEdges.push_back(edge);
    if (outgoingEdgesByNode.find(edge->getDest()) == outgoingEdgesByNode.end())
    {
        outgoingEdgesByNode[edge->getDest()] = vector<Edge<EdgeT> *>();
    }
    outgoingEdgesByNode[edge->getDest()].push_back(edge);
    return true;
}

template <typename NodeT>
bool Node<NodeT>::addEdge(Edge<EdgeT> *edge)
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
        throw "Edge<EdgeT> does not connect to this node";
    }
    return true;
}
template <typename NodeT>
void Node<NodeT>::resetNode()
{
    found = false;
    distance = numeric_limits<double>::infinity();
    previousEdge = nullptr;
    auxIncomingEdges.clear();
    auxOutgoingEdges.clear();
}
template <typename NodeT>
bool Node<NodeT>::isFound()
{
    return found;
}

template <typename NodeT>
void Node<NodeT>::find()
{
    found = true;
}

template <typename NodeT>
double Node<NodeT>::getNodeDistance()
{
    return distance;
}

template <typename NodeT>
void Node<NodeT>::setNodeDistance(double distance)
{
    this->distance = distance;
}

template <typename NodeT, typename EdgeT>
Edge<EdgeT> *Node<NodeT>::getPreviousEdge()
{
    return previousEdge;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT>::setPreviousEdge(Edge<EdgeT> *edge)
{
    this->previousEdge = edge;
    return edge;
}

template <typename NodeT, typename EdgeT>
set<Edge<EdgeT> *> Node<NodeT>::getAuxIncomingEdges()
{
    return auxIncomingEdges;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT>::addAuxIncomingEdge(Edge<EdgeT> *edge)
{
    this->auxIncomingEdges.add(edge);
}

template <typename NodeT, typename EdgeT>
void Node<NodeT>::removeAuxIncomingEdge(Edge<EdgeT> *edge)
{
    this->auxIncomingEdges.remove(edge);
}

template <typename NodeT, typename EdgeT>
set<Edge<EdgeT> *> Node<NodeT>::getAuxOutgoingEdges()
{
    return auxOutgoingEdges;
}

template <typename NodeT, typename EdgeT>
void Node<NodeT>::addAuxOutgoingEdge(Edge<EdgeT> *edge)
{
    this->auxOutgoingEdges.add(edge);
}

template <typename NodeT, typename EdgeT>
void Node<NodeT>::removeAuxOutgoingEdge(Edge<EdgeT> *edge)
{
    this->auxOutgoingEdges.remove(edge);
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Node<NodeT>::getIncomingEdges()
{
    return incomingEdges;
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Node<NodeT>::getOutgoingEdges()
{
    return outgoingEdges;
}

template <typename NodeT, typename EdgeT>
map<Node<NodeT> *, vector<Edge<EdgeT> *>> Node<NodeT>::getIncomingEdgesByNode()
{
    return incomingEdgesByNode;
}

template <typename NodeT, typename EdgeT>
map<Node<NodeT> *, vector<Edge<EdgeT> *>> Node<NodeT>::getOutgoingEdgesByNode()
{
    return outgoingEdgesByNode;
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Node<NodeT>::getIncomingEdgesFromNode(Node<NodeT> *node)
{
    return incomingEdgesByNode[node];
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Node<NodeT>::getOutgoingEdgesToNode(Node<NodeT> *node)
{
    return outgoingEdgesByNode[node];
}