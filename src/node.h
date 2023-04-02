#ifndef NODE_H
#define NODE_H

#include <vector>
#include <map>

using namespace std;
class Edge;

template <class T>
class Node
{
private:
    T data;
    vector<Edge *> incomingEdges;
    map<Node<T> *, vector<Edge *>> incomingEdgesByNode;
    vector<Edge *> outgoingEdges;
    map<Node<T> *, vector<Edge *>> outgoingEdgesByNode;
    bool addIncomingEdge(Edge *edge);
    bool addOutgoingEdge(Edge *edge);

    double distance;
    Edge *previousEdge;

public:
    Node(T data);

    T &getData();
    bool addEdge(Edge *edge);

    void resetNode();
    double getNodeDistance();
    void setNodeDistance(double distance);
    Edge *getPreviousEdge();
    void setPreviousEdge(Edge *edge);

    vector<Edge *> getIncomingEdges();
    vector<Edge *> getOutgoingEdges();
    map<Node<T> *, vector<Edge *>> getIncomingEdgesByNode();
    map<Node<T> *, vector<Edge *>> getOutgoingEdgesByNode();
    vector<Edge *> getIncomingEdgesFromNode(Node<T> *node);
    vector<Edge *> getOutgoingEdgesToNode(Node<T> *node);
};

template <class T>
class NodeDistanceComparator
{
public:
    bool operator()(Node<T> *a, Node<T> *b)
    {
        return a->getNodeDistance() < b->getNodeDistance();
    }
};
#endif