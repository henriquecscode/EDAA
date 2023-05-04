#ifndef MULTIGRAPH_H
#define MULTIGRAPH_H

#include "airport.h"
#include "flight.h"
#include "attribute_type.h"
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
    vector<Edge *> dijkstraShortestPath(Node *source, Node *dest, EdgeFilter edgeFilter, EdgeWeighter edgeWeight);
    vector<Edge *> dijkstraShortestPathEdgesByNode(Node *source, Node *dest, EdgeFilter edgeFilter, EdgeWeighter edgeWeight);
    vector<Edge *> buildPath(Node *source, Node *dest);
    vector<Edge *> bfs(
        Node *n1,
        Node *n2,
        EdgeFilter edgeFilter);
    vector<Edge *> bfsByNode(
        Node *n1,
        Node *n2,
        EdgeFilter edgeFilter);
    vector<Edge *> dfs(
        Node *n1,
        Node *n2,
        EdgeFilter edgeFilter);
    vector<Edge *> dfsByNode(
        Node *n1,
        Node *n2,
        EdgeFilter edgeFilter);
    map<Node *, vector<Edge *>> dfs(
        Node *n1,
        EdgeFilter edgeFilter);
    map<Node *, vector<Edge *>> dfsByNode(
        Node *n1,
        EdgeFilter edgeFilter);
    vector<Edge *> getEdges(EdgeFilter edgeFilter, EdgeWeighter edgeWeight);
    vector<Edge *> getBestEdges(EdgeFilter edgeFilter, EdgeWeighter edgeWeight);
    vector<Edge *> getBestEdgesByNode(Node *node, EdgeFilter edgeFilter, EdgeWeighter edgeWeight);
    bool isConnected(Node *n1, EdgeFilter edgeFilter, map<Node *, vector<Edge *>> (Multigraph::*dfs)(Node *, EdgeFilter));
    void mountTree(Node *root, vector<Edge *> treeEdges);

public:
    Multigraph();
    Node *createNode(Airport data);
    bool createEdge(Node *source, Node *dest, Flight data);

    vector<Node *> getNodes();
    vector<Edge *> getEdges();

    vector<vector<Edge *>> getShortestPathDijkstra(
        vector<Node *> nodes,
        EdgeFilter edgeFilter,
        EdgeWeighter edgeWeight,
        vector<Edge *> (Multigraph::*dijkstra)(Node *, Node *, EdgeFilter, EdgeWeighter));
    vector<vector<Edge *>> getShortestPathDijkstra(
        vector<Node *> nodes,
        EdgeFilter edgeFilter,
        EdgeWeighter edgeWeight,
        int algorithm);

    pair<vector<Edge *>, int> getErdos(
        Node *n1,
        Node *n2,
        EdgeFilter edgeFilter,
        std::vector<Edge *> (Multigraph::*chosenBfs)(Node *n1, Node *n2, EdgeFilter edgeFilter));
    pair<vector<Edge *>, int> getErdos(
        Node *n1,
        Node *n2,
        EdgeFilter edgeFilter,
        int bfsSelection);

    // Local because we just check connectivity from the source node
    void getLocalMinimumSpanningTree(
        Node *localNode,
        EdgeFilter edgeFilter,
        EdgeWeighter edgeWeight,
        std::vector<Edge *> (Multigraph::*chosenCollectEdges)(EdgeFilter edgeFilter, EdgeWeighter edgeWeight),
        map<Node *, vector<Edge *>> (Multigraph::*chosenDfs)(Node *localNode, EdgeFilter edgeFilter));
    // function<vector<Edge *>(EdgeFilter, EdgeWeighter)> collectEdges)
    // function<vector<Edge *>(Node *, EdgeFilter)> dfs);
    // vector<Edge *> (*collectEdges)(EdgeFilter edgeFilter, EdgeWeighter edgeWeight),
    // vector<Edge *> (*dfs)(Node *, EdgeFilter));

    void getLocalMinimumSpanningTree(
        Node *localNode,
        EdgeFilter edgeFilter,
        EdgeWeighter edgeWeight,
        int algorithmCodification);

    vector<vector<Edge *>> getDfs(
        vector<Node *> nodes,
        EdgeFilter edgeFilter,
        EdgeWeighter edgeWeight,
        int dfsAlgorithm);

    Node *getNode(int id);
};
#endif