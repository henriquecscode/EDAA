#include "edge.h"

using namespace std;

template <class T>
Edge<T>::Edge(Node *source, Node *dest, T data)
{
    this->source = source;
    this->dest = dest;
    this->data = data;
}

template <class T>
Node *Edge<T>::getSource()
{
    return source;
}

template <class T>
Node *Edge<T>::getDest()
{
    return dest;
}

template <class T>
T &Edge<T>::getData()
{
    return data;
}