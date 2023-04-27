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
    vector<Edge *> dijkstraShortestPath(Node *source, Node *dest, function<bool(Edge *)> edgeFilter, function<double(Edge*, double&)> edgeWeight);
    vector<Edge *> dijkstraShortestPathEdgesByNode(Node *source, Node *dest, function<bool(Edge *)> edgeFilter, function<double(Edge*, double&)> edgeWeight);
    vector<Edge *> buildPath(Node *source, Node *dest);
    vector<Edge *> bfs(
        Node *n1,
        Node *n2,
        function<bool(Edge *)> edgeFilter);
    vector<Edge *> bfsByNode(
        Node *n1,
        Node *n2,
        function<bool(Edge *)> edgeFilter);
    vector<Edge *> dfs(
        Node *n1,
        Node *n2,
        function<bool(Edge *)> edgeFilter);
    vector<Edge *> dfsByNode(
        Node *n1,
        Node *n2,
        function<bool(Edge *)> edgeFilter);
    map<Node *, vector<Edge *>> dfs(
        Node *n1,
        function<bool(Edge *)> edgeFilter);
    map<Node *, vector<Edge *>> dfsByNode(
        Node *n1,
        function<bool(Edge *)> edgeFilter);
    vector<Edge *> getEdges(function<bool(Edge *)> edgeFilter, function<double(Edge*, double&)> edgeWeight);
    vector<Edge *> getBestEdgesByNode(Node *node, function<bool(Edge *)> edgeFilter, function<double(Edge*, double&)> edgeWeight);
    bool isConnected(Node *n1, function<bool(Edge *)> edgeFilter, vector<Edge *> (*dfs)(Node *, function<bool(Edge*)>));
    void mountTree(Node *root, vector<Edge *> treeEdges);

public:
    Multigraph();
    Node *createNode(Airport data);
    bool createEdge(Node *source, Node *dest, Flight data);

    vector<Node *> getNodes();
    vector<Edge *> getEdges();
    vector<Edge *> getBestEdges(function<bool(Edge *)> edgeFilter, function<double(Edge*, double&)> edgeWeight);

    vector<vector<Edge *>> getShortestPathDijkstra(
        vector<Node *> nodes,
        function<bool(Edge *)> edgeFilter,
        function<double(Edge *)> edgeWeight,
        vector<Edge *> (*dijkstra)(Node *, Node *, function<bool(Edge *)>, function<double(Edge *)>));

    pair<vector<Edge *>, int> getErdos(
        Node *n1,
        Node *n2,
        function<bool(Edge *)> edgeFilter,
        std::vector<Edge *, std::allocator<Edge *>> (Multigraph::*)(Node *n1, Node *n2, std::function<bool(Edge *)> edgeFilter));
    pair<vector<Edge *>, int> getErdos(
        Node *n1,
        Node *n2,
        function<bool(Edge *)> edgeFilter,
        int bfsSelection);

    // Local because we just check connectivity from the source node
    void getLocalMinimumSpanningTree(
        Node *localNode,
        function<bool(Edge *)> edgeFilter,
        function<double(Edge*, double&)> edgeWeight,
        vector<Edge *> (*collectEdges)(function<bool(Edge *)> edgeFilter, function<double(Edge*, double&)> edgeWeight),
        vector<Edge *> (*dfs)(Node *, function<bool(Edge *)>));

    Node *getNode(int id);
};
#endif