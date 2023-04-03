#ifndef NODE_H
#define NODE_H

#include <vector>
#include <map>
#include <set>
using namespace std;

template <typename NodeT, typename EdgeT>
class Edge;

template <typename NodeT, typename EdgeT>
class Node
{
private:
    NodeT data;
    vector<Edge<NodeT, EdgeT> *> incomingEdges;
    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> incomingEdgesByNode;
    vector<Edge<NodeT, EdgeT> *> outgoingEdges;
    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> outgoingEdgesByNode;
    bool addIncomingEdge(Edge<NodeT, EdgeT> *edge);
    bool addOutgoingEdge(Edge<NodeT, EdgeT> *edge);

    bool found;
    double distance;
    Edge<NodeT, EdgeT> *previousEdge;
    set<Edge<NodeT, EdgeT> *> auxIncomingEdges = set<Edge<NodeT, EdgeT> *>();
    set<Edge<NodeT, EdgeT> *> auxOutgoingEdges = set<Edge<NodeT, EdgeT> *>();

public:
    Node(NodeT data);

    NodeT &getData();
    bool addEdge(Edge<NodeT, EdgeT> *edge);

    void resetNode();
    bool isFound();
    void find();
    double getNodeDistance();
    void setNodeDistance(double distance);
    Edge<NodeT, EdgeT> *getPreviousEdge();
    void setPreviousEdge(Edge<NodeT, EdgeT> *edge);
    set<Edge<NodeT, EdgeT> *> getAuxIncomingEdges();
    void addAuxIncomingEdge(Edge<NodeT, EdgeT> *edge);
    void removeAuxIncomingEdge(Edge<NodeT, EdgeT> *edge);
    set<Edge<NodeT, EdgeT> *> getAuxOutgoingEdges();
    void addAuxOutgoingEdge(Edge<NodeT, EdgeT> *edge);
    void removeAuxOutgoingEdge(Edge<NodeT, EdgeT> *edge);

    vector<Edge<NodeT, EdgeT> *> getIncomingEdges();
    vector<Edge<NodeT, EdgeT> *> getOutgoingEdges();
    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> getIncomingEdgesByNode();
    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> getOutgoingEdgesByNode();
    vector<Edge<NodeT, EdgeT> *> getIncomingEdgesFromNode(Node<NodeT, EdgeT> *node);
    vector<Edge<NodeT, EdgeT> *> getOutgoingEdgesToNode(Node<NodeT, EdgeT> *node);
};

template <class NodeT>
class NodeDistanceComparator
{
public:
    bool operator()(Node<NodeT, EdgeT> *a, Node<NodeT, EdgeT> *b)
    {
        return a->getNodeDistance() < b->getNodeDistance();
    }
};
#endif