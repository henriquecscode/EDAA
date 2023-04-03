#ifndef MULTIGRAPH_H
#define MULTIGRAPH_H

#include <vector>
#include <utility>
#include <map>


using namespace std;

template <typename NodeT, typename EdgeT>
class Multigraph
{
    vector<Node<NodeT> *> nodes;
    vector<Edge<EdgeT> *> edges;
    vector<Edge<EdgeT> *> dijkstraShortestPath(Node<NodeT> *source, Node<NodeT> *dest, bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> dijkstraShortestPathEdgesByNode(Node<NodeT> *source, Node<NodeT> *dest, bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> buildPath(Node<NodeT> *source, Node<NodeT> *dest);
    vector<Edge<EdgeT> *> bfs(
        Node<NodeT> *n1,
        Node<NodeT> *n2,
        bool (*edgeFilter)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> bfsByNode(
        Node<NodeT> *n1,
        Node<NodeT> *n2,
        bool (*edgeFilter)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> dfs(
        Node<NodeT> *n1,
        Node<NodeT> *n2,
        bool (*edgeFilter)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> dfsByNode(
        Node<NodeT> *n1,
        Node<NodeT> *n2,
        bool (*edgeFilter)(Edge<EdgeT> *));
    map<Node<NodeT> *, vector<Edge<EdgeT> *>> dfs(
        Node<NodeT> *n1,
        bool (*edgeFilter)(Edge<EdgeT> *));
    map<Node<NodeT> *, vector<Edge<EdgeT> *>> dfsByNode(
        Node<NodeT> *n1,
        bool (*edgeFilter)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> getEdges(bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> getBestEdges(bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *));
    vector<Edge<EdgeT> *> getBestEdgesByNode(Node<NodeT> *node, bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *));
    bool isConnected(Node<NodeT> *n1, bool (*edgeFilter)(Edge<EdgeT> *), vector<Edge<EdgeT> *> (*dfs)(Node<NodeT> *));
    void mountTree(Node<NodeT> *root, vector<Edge<EdgeT> *> treeEdges);

public:
    Multigraph();
    bool createNode(NodeT data);
    bool createEdge(Node<NodeT> *source, Node<NodeT> *dest, EdgeT data);

    void *getNodes();
    void *getEdges();

    vector<vector<Edge<EdgeT> *>> getShortestPathDijkstra(
        vector<Node<NodeT> *> nodes,
        bool (*edgeFilter)(Edge<EdgeT> *),
        double (*edgeWeight)(Edge<EdgeT> *),
        vector<Edge<EdgeT> *> (*dijkstra)(Node<NodeT> *, Node<NodeT> *, bool (*)(Edge<EdgeT> *), double (*)(Edge<EdgeT> *)));

    pair<vector<Edge<EdgeT> *>, int> getErdos(
        Node<NodeT> *n1,
        Node<NodeT> *n2,
        bool (*edgeFilter)(Edge<EdgeT> *),
        vector<Edge<EdgeT> *> (*bfs)(Node<NodeT> *, Node<NodeT> *, bool (*)(Edge<EdgeT> *)));

    // Local because we just check connectivity from the source node
    void getLocalMinimumSpanningTree(
        Node<NodeT> *localNode,
        bool (*edgeFilter)(Edge<EdgeT> *),
        double (*edgeWeight)(Edge<EdgeT> *),
        vector<Edge<EdgeT> *> (*collectEdges)(bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *)),
        vector<Edge<EdgeT> *> (*dfs)(Node<NodeT> *, Node<NodeT> *, bool (*)(Edge<EdgeT> *)));
};
#endif