#ifndef NODE_H
#define NODE_H

#include <vector>
#include <map>
#include <set>
#include "airport.h"
using namespace std;

class Edge;

class Node
{
private:
    Airport data;
    vector<Edge *> incomingEdges;
    map<Node *, vector<Edge *>> incomingEdgesByNode;
    vector<Edge *> outgoingEdges;
    map<Node *, vector<Edge *>> outgoingEdgesByNode;
    bool addIncomingEdge(Edge *edge);
    bool addOutgoingEdge(Edge *edge);

    bool found;
    double distance;
    Edge *previousEdge;
    set<Edge *> auxIncomingEdges = set<Edge *>();
    set<Edge *> auxOutgoingEdges = set<Edge *>();

public:
    Node(Airport data);

    Airport &getData();
    bool addEdge(Edge *edge);

    void resetNode();
    void resetVirtualNode();
    bool isFound();
    void find();
    string toString();
    void print();
    double getNodeDistance();
    void setNodeDistance(double distance);
    Edge *getPreviousEdge();
    void setPreviousEdge(Edge *edge);
    set<Edge *> getAuxIncomingEdges();
    void addAuxIncomingEdge(Edge *edge);
    void removeAuxIncomingEdge(Edge *edge);
    set<Edge *> getAuxOutgoingEdges();
    void addAuxOutgoingEdge(Edge *edge);
    void removeAuxOutgoingEdge(Edge *edge);

    vector<Edge *> getIncomingEdges();
    vector<Edge *> getOutgoingEdges();
    map<Node *, vector<Edge *>> getIncomingEdgesByNode();
    map<Node *, vector<Edge *>> getOutgoingEdgesByNode();
    vector<Edge *> getIncomingEdgesFromNode(Node *node);
    vector<Edge *> getOutgoingEdgesToNode(Node *node);
};

class NodeDistanceComparator
{
public:
    bool operator()(Node *a, Node *b)
    {
        bool smaller = !!(a->getNodeDistance() < b->getNodeDistance());
        return smaller;
    }
};
class NodeDistanceComparatorPair
{
public:
    bool operator()(pair<double, Node *> a, pair<double, Node *> b)
    {
        bool smaller = !!(a.first < b.first);
        return smaller;
    }
};
#endif