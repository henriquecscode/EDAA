#ifndef MULTIGRAPH_H
#define MULTIGRAPH_H

#include <vector>

using namespace std;
template <class T>class Node;
template <class T>
class Edge;

template<class NodeT, class EdgeT>
class Multigraph
{
    vector<Node<NodeT> *> nodes;
    vector<Edge<EdgeT> *> edges;

public:
    Multigraph();
    bool createNode(NodeT data);
    bool createEdge(Node<NodeT> *source, Node<NodeT> *dest, EdgeT data);

    void *getNodes();
    void *getEdges();
};
#endif