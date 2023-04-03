#ifndef NODE_H
#define NODE_H

#include <vector>
#include <map>
#include <set>
using namespace std;

template <typename NodeT>
class Node
{
private:
    NodeT data;
    vector<Edge<EdgeT> *> incomingEdges;
    map<Node<NodeT> *, vector<Edge<EdgeT> *>> incomingEdgesByNode;
    vector<Edge<EdgeT> *> outgoingEdges;
    map<Node<NodeT> *, vector<Edge<EdgeT> *>> outgoingEdgesByNode;
    bool addIncomingEdge(Edge<EdgeT> *edge);
    bool addOutgoingEdge(Edge<EdgeT> *edge);

    bool found;
    double distance;
    Edge<EdgeT> *previousEdge;
    set<Edge<EdgeT> *> auxIncomingEdges = set<Edge<EdgeT> *>();
    set<Edge<EdgeT> *> auxOutgoingEdges = set<Edge<EdgeT> *>();

public:
    Node(NodeT data);

    NodeT &getData();
    bool addEdge(Edge<EdgeT> *edge);

    void resetNode();
    bool isFound();
    void find();
    double getNodeDistance();
    void setNodeDistance(double distance);
    Edge<EdgeT> *getPreviousEdge();
    void setPreviousEdge(Edge<EdgeT> *edge);
    set<Edge<EdgeT> *> getAuxIncomingEdges();
    void addAuxIncomingEdge(Edge<EdgeT> *edge);
    void removeAuxIncomingEdge(Edge<EdgeT> *edge);
    set<Edge<EdgeT> *> getAuxOutgoingEdges();
    void addAuxOutgoingEdge(Edge<EdgeT> *edge);
    void removeAuxOutgoingEdge(Edge<EdgeT> *edge);

    vector<Edge<EdgeT> *> getIncomingEdges();
    vector<Edge<EdgeT> *> getOutgoingEdges();
    map<Node<NodeT> *, vector<Edge<EdgeT> *>> getIncomingEdgesByNode();
    map<Node<NodeT> *, vector<Edge<EdgeT> *>> getOutgoingEdgesByNode();
    vector<Edge<EdgeT> *> getIncomingEdgesFromNode(Node<NodeT> *node);
    vector<Edge<EdgeT> *> getOutgoingEdgesToNode(Node<NodeT> *node);
};

template <class NodeT>
class NodeDistanceComparator
{
public:
    bool operator()(Node<NodeT> *a, Node<NodeT> *b)
    {
        return a->getNodeDistance() < b->getNodeDistance();
    }
};
#endif