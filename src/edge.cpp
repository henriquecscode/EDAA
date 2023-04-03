#include "edge.h"
using namespace std;

template <typename EdgeT>
Edge<EdgeT>::Edge(Node<NodeT> *source, Node<NodeT> *dest, EdgeT data)
{
    this->source = source;
    this->dest = dest;
    this->data = data;
}

template <typename EdgeT>
Node<NodeT> *Edge<EdgeT>::getSource()
{
    return source;
}

template <typename EdgeT>
Node<NodeT> *Edge<EdgeT>::getDest()
{
    return dest;
}

template <typename EdgeT>
EdgeT &Edge<EdgeT>::getData()
{
    return data;
}