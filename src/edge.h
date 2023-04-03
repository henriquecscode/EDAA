#ifndef EDGE_H
#define EDGE_H

template <typename NodeT>
class Node;

template <typename EdgeT>
class Edge
{
    Node<NodeT> *source;
    Node<NodeT> *dest;
    EdgeT data;
public:
    Edge(Node<NodeT> *source, Node<NodeT> *dest, EdgeT data);
    Node<NodeT> *getSource();
    Node<NodeT> *getDest();
    EdgeT &getData();
};
#endif
