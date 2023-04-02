#ifndef NODE_H
#define NODE_H

#include <vector>

using namespace std;
class Edge;

template <class T>
class Node
{
private:
    T data;
    vector<Edge *> incomingEdges;
    vector<Edge *> outgoingEdges;
    bool addIncomingEdge(Edge *edge);
    bool addOutgoingEdge(Edge *edge);
    double distance;
    Edge* previousEdge;

public:
    Node(T data);

    T &getData();
    bool addEdge(Edge *edge);

    void resetNode();
    double getNodeDistance();
    void setNodeDistance(double distance);
    Edge *getPreviousEdge();
    void setPreviousEdge(Edge *edge);
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