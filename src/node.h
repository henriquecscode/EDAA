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

public:
    Node(T data);

    T &getData();
    bool addEdge(Edge *edge);
};
#endif