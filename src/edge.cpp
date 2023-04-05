#include "edge.h"
#include <string>
#include <functional>
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

template <typename NodeT, typename EdgeT>
function<bool(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeFilter(double (*f)(EdgeT *), double min, double max)
{
    return [f, min, max](Edge<NodeT, EdgeT> *edge) -> bool
    {
        double value = f(&(edge->getData()));
        return min <= value && value <= max;
    };
}

template <typename NodeT, typename EdgeT>
function<bool(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeFilter(double (*f)(EdgeT *), int min, int max)
{
    return [f, min, max](Edge<NodeT, EdgeT> *edge) -> bool
    {
        int value = f(&(edge->getData()));
        return min <= value && value <= max;
    };
}

template <typename NodeT, typename EdgeT>
function<bool(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeFilter(double (*f)(EdgeT *), string comparison)
{
    return [f, comparison](Edge<NodeT, EdgeT> *edge) -> bool
    {
        string value = f(&(edge->getData()));
        return value == comparison;

    };
}