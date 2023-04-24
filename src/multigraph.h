#ifndef MULTIGRAPH_H
#define MULTIGRAPH_H

#include "airport.h"
#include "flight.h"
#include <vector>
#include <utility>
#include <map>

using namespace std;

class Node;

class Edge;

class Multigraph
{
    vector<Node *> nodes;
    vector<Edge *> edges;
    vector<Edge *> dijkstraShortestPath(Node *source, Node *dest, bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *));
    vector<Edge *> dijkstraShortestPathEdgesByNode(Node *source, Node *dest, bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *));
    vector<Edge *> buildPath(Node *source, Node *dest);
    vector<Edge *> bfs(
        Node *n1,
        Node *n2,
        bool (*edgeFilter)(Edge *));
    vector<Edge *> bfsByNode(
        Node *n1,
        Node *n2,
        bool (*edgeFilter)(Edge *));
    vector<Edge *> dfs(
        Node *n1,
        Node *n2,
        bool (*edgeFilter)(Edge *));
    vector<Edge *> dfsByNode(
        Node *n1,
        Node *n2,
        bool (*edgeFilter)(Edge *));
    map<Node *, vector<Edge *>> dfs(
        Node *n1,
        bool (*edgeFilter)(Edge *));
    map<Node *, vector<Edge *>> dfsByNode(
        Node *n1,
        bool (*edgeFilter)(Edge *));
    vector<Edge *> getEdges(bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *));
    vector<Edge *> getBestEdgesByNode(Node *node, bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *));
    bool isConnected(Node *n1, bool (*edgeFilter)(Edge *), vector<Edge *> (*dfs)(Node *, bool (*)(Edge *)));
    void mountTree(Node *root, vector<Edge *> treeEdges);

public:
    Multigraph();
    Node *createNode(Airport data);
    bool createEdge(Node *source, Node *dest, Flight data);

    vector<Node *> getNodes();
    vector<Edge *> getEdges();
    vector<Edge *> getBestEdges(function<bool(Edge *)> *edgeFilter, function<double(Edge *)> *edgeWeight);

    vector<vector<Edge *>> getShortestPathDijkstra(
        vector<Node *> nodes,
        bool (*edgeFilter)(Edge *),
        double (*edgeWeight)(Edge *),
        vector<Edge *> (*dijkstra)(Node *, Node *, bool (*)(Edge *), double (*)(Edge *)));

    pair<vector<Edge *>, int> getErdos(
        Node *n1,
        Node *n2,
        bool (*edgeFilter)(Edge *),
        vector<Edge *> (*bfs)(Node *, Node *, bool (*)(Edge *)));

    // Local because we just check connectivity from the source node
    void getLocalMinimumSpanningTree(
        Node *localNode,
        bool (*edgeFilter)(Edge *),
        double (*edgeWeight)(Edge *),
        vector<Edge *> (*collectEdges)(bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *)),
        vector<Edge *> (*dfs)(Node *, bool (*)(Edge *)));

    Node *getNode(int id);
};
#endif