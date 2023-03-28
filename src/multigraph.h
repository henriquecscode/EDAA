#ifndef MULTIGRAPH_H
#define MULTIGRAPH_H

class Node;
class Edge;
template <class NodeT, class EdgeT>
class Multigraph
{    vector<Node *> nodes;
    vector<Edge *> edges;

public:
    Multigraph();
    bool createNode(NodeT data);
    bool createEdge(Node *source, Node *dest, EdgeT data);

    void *getNodes();
    void *getEdges();
};
#endif