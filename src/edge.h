#ifndef EDGE_H
#define EDGE_H

class Node;

template <class T>
class Edge
{
private:
public:
    Edge(Node *source, Node *dest, T data);
    Node *getSource();
    Node *getDest();
    T &getData();
};
#endif
