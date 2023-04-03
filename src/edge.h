#ifndef EDGE_H
#define EDGE_H

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
};
#endif
