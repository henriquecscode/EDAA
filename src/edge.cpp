#include "edge.h"

class Node;

template <class T>
class Edge
{
private:
    Node *source;
    Node *dest;
    T data;

public:
    Edge(Node *source, Node *dest, T data)
    {
        this->source = source;
        this->dest = dest;
        this->data = data;
    }

    Node *getSource()
    {
        return source;
    }

    Node *getDest()
    {
        return dest;
    }

    T &getData()
    {
        return data;
    }
};