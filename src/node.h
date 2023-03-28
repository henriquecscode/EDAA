#ifndef NODE_H
#define NODE_H

class Edge;

template <class T>
class Node
{
private:
    bool addIncomingEdge(Edge *edge);
    bool addOutgoingEdge(Edge *edge);

public:
    Node(T data);

    T &getData();
    bool addEdge(Edge *edge);
};
#endif