#ifndef MULTIGRAPH_H
#define MULTIGRAPH_H

#include <vector>
#include <utility>
#include <map>

using namespace std;

template <typename NodeT, typename EdgeT>
class Node;

template <typename NodeT, typename EdgeT>
class Edge;

template <typename NodeT, typename EdgeT>
class Multigraph
{
    vector<Node<NodeT, EdgeT> *> nodes;
    vector<Edge<NodeT, EdgeT> *> edges;
    vector<Edge<NodeT, EdgeT> *> dijkstraShortestPath(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> dijkstraShortestPathEdgesByNode(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> buildPath(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest);
    vector<Edge<NodeT, EdgeT> *> bfs(
        Node<NodeT, EdgeT> *n1,
        Node<NodeT, EdgeT> *n2,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> bfsByNode(
        Node<NodeT, EdgeT> *n1,
        Node<NodeT, EdgeT> *n2,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> dfs(
        Node<NodeT, EdgeT> *n1,
        Node<NodeT, EdgeT> *n2,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> dfsByNode(
        Node<NodeT, EdgeT> *n1,
        Node<NodeT, EdgeT> *n2,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *));
    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> dfs(
        Node<NodeT, EdgeT> *n1,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *));
    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> dfsByNode(
        Node<NodeT, EdgeT> *n1,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> getEdges(bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> getBestEdges(bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *));
    vector<Edge<NodeT, EdgeT> *> getBestEdgesByNode(Node<NodeT, EdgeT> *node, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *));
    bool isConnected(Node<NodeT, EdgeT> *n1, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), vector<Edge<NodeT, EdgeT> *> (*dfs)(Node<NodeT, EdgeT> *));
    void mountTree(Node<NodeT, EdgeT> *root, vector<Edge<NodeT, EdgeT> *> treeEdges);

public:
    Multigraph();
    Node<NodeT, EdgeT> createNode(NodeT data);
    bool createEdge(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, EdgeT data);

    void *getNodes();
    void *getEdges();

    vector<vector<Edge<NodeT, EdgeT> *>> getShortestPathDijkstra(
        vector<Node<NodeT, EdgeT> *> nodes,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *),
        double (*edgeWeight)(Edge<NodeT, EdgeT> *),
        vector<Edge<NodeT, EdgeT> *> (*dijkstra)(Node<NodeT, EdgeT> *, Node<NodeT, EdgeT> *, bool (*)(Edge<NodeT, EdgeT> *), double (*)(Edge<NodeT, EdgeT> *)));

    pair<vector<Edge<NodeT, EdgeT> *>, int> getErdos(
        Node<NodeT, EdgeT> *n1,
        Node<NodeT, EdgeT> *n2,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *),
        vector<Edge<NodeT, EdgeT> *> (*bfs)(Node<NodeT, EdgeT> *, Node<NodeT, EdgeT> *, bool (*)(Edge<NodeT, EdgeT> *)));

    // Local because we just check connectivity from the source node
    void getLocalMinimumSpanningTree(
        Node<NodeT, EdgeT> *localNode,
        bool (*edgeFilter)(Edge<NodeT, EdgeT> *),
        double (*edgeWeight)(Edge<NodeT, EdgeT> *),
        vector<Edge<NodeT, EdgeT> *> (*collectEdges)(bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *)),
        vector<Edge<NodeT, EdgeT> *> (*dfs)(Node<NodeT, EdgeT> *, Node<NodeT, EdgeT> *, bool (*)(Edge<NodeT, EdgeT> *)));
};
#endif