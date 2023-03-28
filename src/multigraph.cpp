#include "multigraph.h"
#include <vector>
using namespace std;

template <class NodeT, class EdgeT>
Multigraph<NodeT, EdgeT>::Multigraph()
{
    nodes = vector<Node<NodeT> *>();
    edges = vector<Edge<EdgeT> *>();
};

template <class NodeT, class EdgeT>
bool Multigraph<NodeT, EdgeT>::createNode(NodeT data)
{
    Node<NodeT> *node = new Node<NodeT>(data);
    this->nodes.push_back(node);
    return true;
}

template <class NodeT, class EdgeT>
bool Multigraph<NodeT, EdgeT>::createEdge(Node<NodeT> *source, Node<NodeT> *dest, EdgeT data)
{
    Edge<EdgeT> *edge = new Edge<EdgeT>(source, dest, data);
    this->edges.push_back(edge);
    source->addEdge(edge);
    return true;
}

template <class NodeT, class EdgeT>
void *Multigraph<NodeT, EdgeT>::getNodes()
{
    return &(this->nodes);
}

template <class NodeT, class EdgeT>
void *Multigraph<NodeT, EdgeT>::getEdges()
{
    return &(this->edges);
}