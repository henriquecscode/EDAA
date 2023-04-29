#include "multigraph.h"
#include "node.h"
#include "edge.h"
#include "airport.h"
#include "flight.h"
#include "attribute_type.h"
#include <vector>
#include <queue>
#include <set>
#include <utility>
#include <limits>
#include <algorithm>
#include <stack>
#include <iostream>
#include <cmath>
// #include "better_priority_queue.h" //not in makefile
using namespace std;

void printQueue(priority_queue<Node *, vector<Node *>, NodeDistanceComparator> queue)
{
    Node *node;
    cout << "Queue ";
    while (!queue.empty())
    {
        node = queue.top();
        cout << node->getNodeDistance() << " ";
        queue.pop();
    }
    cout << endl;
}
void printQueue(priority_queue<pair<double, Node *>, vector<pair<double, Node *>>, NodeDistanceComparatorPair> queue)
{
    Node *node;
    double distance;
    pair<double, Node *> pair;
    cout << "Queue ";
    while (!queue.empty())
    {
        pair = queue.top();
        cout << pair.first << ":" << pair.second->getData().getId() << ":" << pair.second->getNodeDistance() << " ";
        // cout << node->getNodeDistance() << " ";
        queue.pop();
    }
    cout << endl;
}

Multigraph::Multigraph()
{
    nodes = vector<Node *>();
    edges = vector<Edge *>();
};

Node *Multigraph::createNode(Airport data)
{
    Node *node = new Node(data);
    this->nodes.push_back(node);
    return node;
}

bool Multigraph::createEdge(Node *source, Node *dest, Flight data)
{
    Edge *edge = new Edge(source, dest, data);
    this->edges.push_back(edge);
    source->addEdge(edge);
    return true;
}

vector<Node *> Multigraph::getNodes()
{
    return this->nodes;
}

vector<Edge *> Multigraph::getEdges()
{
    return this->edges;
}

vector<Edge *> Multigraph::dijkstraShortestPath(Node *source, Node *dest, EdgeFilter edgeFilter, EdgeWeighter edgeWeight)
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    // better_priority_queue::updatable_priority_queue<Node *, double> pQ;
    vector<Edge *> path;
    // priority_queue<Node *, vector<Node *>, NodeDistanceComparator> queue;
    priority_queue<pair<double, Node *>, vector<pair<double, Node *>>, NodeDistanceComparatorPair> queue;

    for (auto node : nodes)
    {
        node->resetNode();
    }
    source->setNodeDistance(0);
    queue.push(make_pair(0, source));
    // pQ.set(source, 0);

    // thank you copilot <3 <3
    // while (!pQ.empty())
    while (!queue.empty())
    {
        // Node *node = pQ.pop_value().key;
        Node *node = queue.top().second;
        cout << "Checking node " << node->getData().getId() << endl;
        cout << "Distance is" << node->getNodeDistance() << endl;
        printQueue(queue);
        queue.pop();
        if (node->isFound())
        {
            continue;
        }
        // check if node is destination
        if (node == dest)
        {
            cout << "----- FOUND DESTINATION -----" << endl;
            break;
        }

        vector<Edge *> outgoingEdges = node->getOutgoingEdges();
        cout << "Node has " << outgoingEdges.size() << "connections" << endl;

        for (auto edge : outgoingEdges)
        {
            if (edgeFilter(edge))
            {
                // cout << "Checking one specific edge " << endl;
                Node *toNode = edge->getDest();
                // double alt = node->getDistance() + edgeWeight(edge);
                double weight;
                edgeWeight(edge, weight);
                double alt = node->getNodeDistance() - weight;
                if (alt > toNode->getNodeDistance() || isinf(toNode->getNodeDistance()))
                {
                    toNode->setNodeDistance(alt);
                    queue.push(make_pair(alt, toNode));
                    // pQ.set(toNode, alt);
                    toNode->setPreviousEdge(edge);
                }
            }
        }
        cout << "Checked the connections" << endl;
        node->find();
    }
    path = buildPath(source, dest);

    return path;
}

vector<Edge *> Multigraph::dijkstraShortestPathEdgesByNode(Node *source, Node *dest, EdgeFilter edgeFilter, EdgeWeighter edgeWeight)
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    // better_priority_queue::updatable_priority_queue<Node *, double> pQ;

    priority_queue<Node *, vector<Node *>, NodeDistanceComparator> queue;

    for (auto node : nodes)
    {
        node->resetNode();
    }
    source->setNodeDistance(0);
    queue.push(source);
    // pQ.set(source, 0);

    // thank you copilot <3
    // while (!pQ.empty())
    while (!queue.empty())
    {
        // Node *node = pQ.pop_value().key;
        // check if node is destination
        Node *node = queue.top();
        cout << "Checking node " << node->getData().getId() << endl;
        cout << "Distance is" << node->getNodeDistance() << endl;
        printQueue(queue);
        queue.pop();
        if (node->isFound())
        {
            continue;
        }
        if (node == dest)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            // get the best edge
            double bestWeight = numeric_limits<double>::max();
            Edge *bestEdge = nullptr;
            Node *toNode = outgoingEdgesOfNode.first;
            vector<Edge *> outgoingEdges = outgoingEdgesOfNode.second;
            cout << "Checking" << outgoingEdges.size() << "edges to " << toNode->getData().getId() << endl;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    double weight;
                    edgeWeight(edge, weight);
                    if (weight < bestWeight)
                    {
                        bestWeight = weight;
                        bestEdge = edge;
                    }
                }
            }

            // use the best edge for dijkstra
            double weight;
            edgeWeight(bestEdge, weight);
            double alt = node->getNodeDistance() - weight;
            if (alt > toNode->getNodeDistance() || isinf(toNode->getNodeDistance()))
            {
                toNode->setNodeDistance(alt);
                queue.push(toNode);
                // pQ.set(toNode, alt);
                toNode->setPreviousEdge(bestEdge);
            }
        }
        node->find();
    }
    vector<Edge *> path = buildPath(source, dest);
    return path;
}

// build path

vector<Edge *> Multigraph::buildPath(Node *source, Node *dest)
{
    vector<Edge *> path;
    Node *node = dest;

    while (node != source)
    {
        path.push_back(node->getPreviousEdge());
        node = node->getPreviousEdge()->getSource();
    }
    reverse(path.begin(), path.end());
    return path;
}

vector<vector<Edge *>> Multigraph::getShortestPathDijkstra(
    vector<Node *> nodes,
    EdgeFilter edgeFilter,
    EdgeWeighter edgeWeight,
    vector<Edge *> (Multigraph::*dijkstra)(Node *, Node *, EdgeFilter, EdgeWeighter))
{
    {
        if (nodes.size() < 2)
        {
            throw "Not enough nodes";
        }
        vector<vector<Edge *>> allPaths;
        for (auto i = 1; i < nodes.size(); i++)
        {
            vector<Edge *> path = (this->*dijkstra)(nodes[i - 1], nodes[i], edgeFilter, edgeWeight);
            allPaths.push_back(path);
        }
        return allPaths;
    }
}

vector<vector<Edge *>> Multigraph::getShortestPathDijkstra(
    vector<Node *> nodes,
    EdgeFilter edgeFilter,
    EdgeWeighter edgeWeight,
    int algorithm)
{
    vector<Edge *> (Multigraph::*chosenDijkstra)(Node *, Node *, EdgeFilter, EdgeWeighter);
    if (algorithm == 1)
    {
        chosenDijkstra = &Multigraph::dijkstraShortestPath;
    }
    else if (algorithm == 2)
    {
        chosenDijkstra = &Multigraph::dijkstraShortestPathEdgesByNode;
    }
    return getShortestPathDijkstra(nodes, edgeFilter, edgeWeight, chosenDijkstra);
}

vector<Edge *> Multigraph::bfs(
    Node *n1,
    Node *n2,
    EdgeFilter edgeFilter)
{
    queue<Node *> q;
    vector<Edge *> path;

    if (n1 == n2)
    {
        return path;
    }

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // bfs over nodes using visited
    q.push(n1);
    n1->find();
    while (!q.empty())
    {
        Node *node = q.front();
        q.pop();
        if (node == n2)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node *toNode = edge->getDest();
                if (!toNode->isFound())
                {
                    // Not found
                    q.push(toNode);
                    toNode->setPreviousEdge(edge);
                    toNode->find();
                }
            }
        }
    }

    path = buildPath(n1, n2);
    return path;
}

vector<Edge *> Multigraph::bfsByNode(
    Node *n1,
    Node *n2,
    EdgeFilter edgeFilter)
{
    queue<Node *> q;
    vector<Edge *> path;

    if (n1 == n2)
    {
        return path;
    }

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // bfs over nodes using visited
    q.push(n1);
    n1->find();
    while (!q.empty())
    {
        Node *node = q.front();
        q.pop();
        if (node == n2)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node *toNode = outgoingEdgesOfNode.first;
            vector<Edge *> outgoingEdges = outgoingEdgesOfNode.second;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    if (!toNode->isFound())
                    {
                        // Not found
                        q.push(toNode);
                        toNode->setPreviousEdge(edge);
                        toNode->find();
                        break;
                    }
                }
            }
        }
    }

    path = buildPath(n1, n2);
    return path;
}

pair<vector<Edge *>, int> Multigraph::getErdos(
    Node *n1,
    Node *n2,
    EdgeFilter edgeFilter,
    std::vector<Edge *> (Multigraph::*chosenBfs)(Node *n1, Node *n2, EdgeFilter edgeFilter))
{
    vector<Edge *> path = (this->*chosenBfs)(n1, n2, edgeFilter);
    return make_pair(path, path.size());
}

pair<vector<Edge *>, int> Multigraph::getErdos(
    Node *n1,
    Node *n2,
    EdgeFilter edgeFilter,
    int bfsSelection)
{
    std::vector<Edge *> (Multigraph::*chosenBfs)(Node * n1, Node * n2, EdgeFilter edgeFilter);
    if (bfsSelection == 1)
    {
        chosenBfs = &Multigraph::bfs;
    }
    else if (bfsSelection == 2)
    {
        chosenBfs = &Multigraph::bfsByNode;
    }
    return this->getErdos(n1, n2, edgeFilter, chosenBfs);
}

vector<Edge *> Multigraph::dfs(
    Node *n1,
    Node *n2,
    EdgeFilter edgeFilter)
{
    stack<Node *> s;
    vector<Edge *> path;

    if (n1 == n2)
    {
        return path;
    }

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node *node = s.top();
        s.pop();
        if (node == n2)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node *toNode = edge->getDest();
                if (!toNode->isFound())
                {
                    // Not found
                    s.push(toNode);
                    toNode->setPreviousEdge(edge);
                    toNode->find();
                }
            }
        }
    }

    path = buildPath(n1, n2);
    return path;
}

map<Node *, vector<Edge *>> Multigraph::dfs(
    Node *n1,
    EdgeFilter edgeFilter)
{
    set<Node *> found;
    stack<Node *> s;
    vector<Edge *> path;

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node *node = s.top();
        s.pop();

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node *toNode = edge->getDest();
                if (found.find(toNode) == found.end())
                {
                    // Not found
                    s.push(toNode);
                    toNode->setPreviousEdge(edge);
                    found.insert(toNode);
                }
            }
        }
    }

    map<Node *, vector<Edge *>> pathMap;
    for (auto node : nodes)
    {
        if (found.find(node) != found.end())
        {
            vector<Edge *> nodePath = buildPath(n1, node);
            pathMap[node] = nodePath;
        }
    }
    return pathMap;
}

vector<Edge *> Multigraph::dfsByNode(
    Node *n1,
    Node *n2,
    EdgeFilter edgeFilter)
{
    stack<Node *> s;
    vector<Edge *> path;

    if (n1 == n2)
    {
        return path;
    }

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node *node = s.top();
        s.pop();
        if (node == n2)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node *toNode = outgoingEdgesOfNode.first;
            vector<Edge *> outgoingEdges = outgoingEdgesOfNode.second;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    if (!toNode->isFound())
                    {
                        // Not found
                        s.push(toNode);
                        toNode->setPreviousEdge(edge);
                        toNode->find();
                        break;
                    }
                }
            }
        }
    }

    path = buildPath(n1, n2);
    return path;
}

map<Node *, vector<Edge *>> Multigraph::dfsByNode(
    Node *n1,
    EdgeFilter edgeFilter)
{
    set<Node *> found;
    stack<Node *> s;

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node *node = s.top();
        s.pop();

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node *toNode = outgoingEdgesOfNode.first;
            vector<Edge *> outgoingEdges = outgoingEdgesOfNode.second;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    if (!toNode->isFound())
                    {
                        // Not found
                        s.push(toNode);
                        toNode->setPreviousEdge(edge);
                        toNode->find();
                    }
                }
            }
        }
    }

    map<Node *, vector<Edge *>> pathMap;
    for (auto node : found)
    {
        vector<Edge *> nodePath = buildPath(n1, node);
        pathMap[node] = nodePath;
    }
    return pathMap;
}

vector<Edge *> Multigraph::getEdges(EdgeFilter edgeFilter, EdgeWeighter edgeWeight)
{
    vector<Edge *> filteredEdges;
    copy_if(this->edges.begin(), this->edges.end(), filteredEdges.begin(), edgeFilter);
    return filteredEdges;
}

vector<Edge *> Multigraph::getBestEdges(EdgeFilter edgeFilter, EdgeWeighter edgeWeight)
{
    // I think this is useles. Will only get the edges with least weight for the local minimum spanning tree
    // We need every edge
    vector<Edge *> bestEdges;
    double bestWeight = numeric_limits<double>::max(); // Initialize to a large number
    for (auto edge : this->edges)
    {
        if (edgeFilter(edge))
        {
            double weight;
            edgeWeight(edge, weight);
            if (weight < bestWeight)
            {
                bestEdges.clear();
                bestEdges.push_back(edge);
                bestWeight = weight;
            }
            else if (weight == bestWeight)
            {
                bestEdges.push_back(edge);
            }
        }
    }
    return bestEdges;
}

vector<Edge *> Multigraph::getBestEdgesByNode(Node *node, EdgeFilter edgeFilter, EdgeWeighter edgeWeight)
{
    vector<Edge *> bestEdges;
    double bestWeight = numeric_limits<double>::max(); // Initialize to a large number
    auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
    for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
    {
        Node *toNode = outgoingEdgesOfNode.first;
        vector<Edge *> outgoingEdges = outgoingEdgesOfNode.second;
        for (auto edge : outgoingEdges)
        {
            if (edgeFilter(edge))
            {
                double weight;
                edgeWeight(edge, weight);
                if (weight < bestWeight)
                {
                    bestEdges.clear();
                    bestEdges.push_back(edge);
                    bestWeight = weight;
                }
            }
        }
    }
    return bestEdges;
}

bool Multigraph::isConnected(Node *n1, EdgeFilter edgeFilter, map<Node *, vector<Edge *>> (Multigraph::*dfs)(Node *, EdgeFilter))
{
    (this->*dfs)(n1, edgeFilter);

    for (auto node : nodes)
    {
        if (!node->isFound())
        {
            return false;
        }
    }
    return true;
}

void Multigraph::mountTree(Node *root, vector<Edge *> treeEdges)
{
    return;
}

void Multigraph::getLocalMinimumSpanningTree(
    Node *localNode,
    EdgeFilter edgeFilter,
    EdgeWeighter edgeWeight,
    std::vector<Edge *> (Multigraph::*chosenCollectEdges)(EdgeFilter edgeFilter, EdgeWeighter edgeWeight),
    map<Node *, vector<Edge *>> (Multigraph::*chosenDfs)(Node *localNode, EdgeFilter edgeFilter))
{

    vector<Edge *> edges = (this->*chosenCollectEdges)(edgeFilter, edgeWeight);
    vector<Edge *> mstEdges = vector<Edge *>();

    // sort edges by weight
    sort(edges.begin(), edges.end(), [edgeWeight](Edge *e1, Edge *e2)
         {
            double w1, w2;
            edgeWeight(e1, w1);
            edgeWeight(e2, w2);
            return w1 < w2; });

    for (auto node : nodes)
    {
        node->resetNode();
    }

    for (auto edge : edges)
    {
        Node *n1 = edge->getSource();
        Node *n2 = edge->getDest();
        n1->addAuxOutgoingEdge(edge);
        n2->addAuxIncomingEdge(edge);
    }

    for (auto edge : edges)
    {
        Node *n1 = edge->getSource();
        Node *n2 = edge->getDest();

        n1->removeAuxOutgoingEdge(edge);
        n2->removeAuxIncomingEdge(edge);

        bool connected = isConnected(
            localNode, [](Edge *e)
            { 
                auto it = e->getSource()->getAuxOutgoingEdges().find(e);
                return it!=e->getSource()->getAuxOutgoingEdges().end(); },
            chosenDfs);

        if (!connected)
        {
            // restitute edge
            n1->addAuxOutgoingEdge(edge);
            n2->addAuxIncomingEdge(edge);
        }
        else
        {
            mstEdges.push_back(edge);
        }
    }

    mountTree(localNode, mstEdges);
}

void Multigraph::getLocalMinimumSpanningTree(
    Node *localNode,
    EdgeFilter edgeFilter,
    EdgeWeighter edgeWeight,
    int collectionAlgorithm,
    int dfsAlgorithm)
{

    std::vector<Edge *> (Multigraph::*chosenCollectEdges)(EdgeFilter edgeFilter, EdgeWeighter edgeWeight);
    map<Node *, vector<Edge *>> (Multigraph::*chosenDfs)(Node * localNode, EdgeFilter edgeFilter);
    if (collectionAlgorithm == 0)
    {
        chosenCollectEdges = &Multigraph::getEdges;
    }
    else
    {
        chosenCollectEdges = &Multigraph::getBestEdges;
    }

    if (dfsAlgorithm == 0)
    {
        chosenDfs = &Multigraph::dfs;
    }
    else
    {
        chosenDfs = &Multigraph::dfsByNode;
    }

    return getLocalMinimumSpanningTree(localNode, edgeFilter, edgeWeight, chosenCollectEdges, chosenDfs);
}

Node *Multigraph::getNode(int id)
{
    auto it = find_if(nodes.begin(), nodes.end(), [id](Node *n)
                      { return n->getData().getId() == id; });
    if (it == nodes.end())
    {
        std::cout << "Could not find node\n";
        return nullptr;
    }
    return *it;
}
