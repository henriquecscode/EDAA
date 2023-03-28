#include "multigraph.h"
#include <vector>
using namespace std;

template <class NodeT, class EdgeT>
Multigraph<NodeT, EdgeT>::Multigraph()
{
    nodes = vector<Node *>();
    edges = vector<Edge *>();
};

template <class NodeT, class EdgeT>
bool Multigraph<NodeT, EdgeT>::createNode(NodeT data)
{
    Node *node = new Node(data);
    nodes.push_back(node);
    return true;
}

template <class NodeT, class EdgeT>
bool Multigraph<NodeT, EdgeT>::createEdge(Node *source, Node *dest, EdgeT data)
{
    Edge *edge = new Edge(source, dest, data);
    edges.push_back(edge);
    source->addEdge(edge);
    return true;
}

template <class NodeT, class EdgeT>
void *Multigraph<NodeT, EdgeT>::getNodes()
{
    return &nodes;
}

template <class NodeT, class EdgeT>
void *Multigraph<NodeT, EdgeT>::getEdges()
{
    return &edges;
}