#ifndef EDGE_H
#define EDGE_H

#include <string>
#include <functional>

using namespace std;

template <typename NodeT, typename EdgeT>
class Node;

template <typename NodeT, typename EdgeT>
class Edge
{
    Node<NodeT, EdgeT> *source;
    Node<NodeT, EdgeT> *dest;
    EdgeT data;
public:
    Edge(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, EdgeT data);
    Node<NodeT, EdgeT> *getSource();
    Node<NodeT, EdgeT> *getDest();
    EdgeT &getData();

    function<bool (Edge<NodeT,EdgeT>*)>getEdgeFilter(double (*f)(EdgeT*), double min, double max);
    function<bool (Edge<NodeT,EdgeT>*)>getEdgeFilter(double (*f)(EdgeT*), int min, int max);
    function<bool (Edge<NodeT,EdgeT>*)>getEdgeFilter(double (*f)(EdgeT*), string comparison);

    function<bool (Edge<NodeT,EdgeT>*)>getEdgeWeight(double (*f)(EdgeT*), string comparison);
};
#endif
