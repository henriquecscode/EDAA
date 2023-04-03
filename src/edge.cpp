#include "edge.h"
using namespace std;

template <typename NodeT, typename EdgeT>
Edge<NodeT, EdgeT>::Edge(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, EdgeT data)
{
    this->source = source;
    this->dest = dest;
    this->data = data;
}

template <typename NodeT, typename EdgeT>
Node<NodeT, EdgeT> *Edge<NodeT, EdgeT>::getSource()
{
    return source;
}

template <typename NodeT, typename EdgeT>
Node<NodeT, EdgeT> *Edge<NodeT, EdgeT>::getDest()
{
    return dest;
}

template <typename NodeT, typename EdgeT>
EdgeT &Edge<NodeT, EdgeT>::getData()
{
    return data;
}