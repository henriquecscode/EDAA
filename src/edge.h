#ifndef EDGE_H
#define EDGE_H

#include <string>
#include <functional>

using namespace std;

enum attributeType
{
    INT,
    DOUBLE,
    STRING
};

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
    EdgeT *getData();

    // function<bool (Edge<NodeT, EdgeT>*)>getEdgeFilter(double (*f)(EdgeT*), double min, double max);

    static function<bool(Edge<NodeT, EdgeT> *)> getEdgeFilter();
    // static function<bool(Edge<NodeT, EdgeT> *)> getEdgeFilter(string dataAttributeName, double min, double max);
    static function<bool(Edge<NodeT, EdgeT> *)> getEdgeFilter(double (EdgeT::*f)(), double min, double max);
    static function<bool(Edge<NodeT, EdgeT> *)> getEdgeFilter(int (EdgeT::*f)(), int min, int max);
    static function<bool(Edge<NodeT, EdgeT> *)> getEdgeFilter(string (EdgeT::*f)(), string comparison);

    static function<double(Edge<NodeT, EdgeT> *)> getEdgeWeight(string dataAttributeName);
    static function<double(Edge<NodeT, EdgeT> *)> getEdgeWeight(double (EdgeT::*f)());
    static function<double(Edge<NodeT, EdgeT> *)> getEdgeWeight(int (EdgeT::*f)());
};
#endif
