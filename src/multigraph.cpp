#include "multigraph.h"
#include "node.h"
#include "edge.h"
#include "airport.h"
#include "flight.h"
#include <vector>
#include <queue>
#include <set>
#include <utility>
#include <limits>
#include <algorithm>
#include <stack>
#include "better_priority_queue.h"
using namespace std;

Multigraph::Multigraph()
{
    nodes = vector<Node *>();
    edges = vector<Edge *>();
};

Node* Multigraph::createNode(Airport data) 
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

vector<Edge *> Multigraph::dijkstraShortestPath(Node *source, Node *dest, bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *))
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    better_priority_queue::updatable_priority_queue<Node *, double> pQ;
    vector<Edge *> path;
    // priority_queue<Node *, vector<Node *>, NodeDistanceComparator> queue;

    for (auto node : nodes)
    {
        node->resetNode();
    }
    // source->setDistance(0);
    // queue.push(source);
    pQ.set(source, 0);

    // thank you copilot <3 <3
    while (!pQ.empty())
    {
        Node *node = pQ.pop_value().key;
        // check if node is destination
        if (node == dest)
        {
            break;
        }

        vector<Edge *> outgoingEdges = node->getOutgoingEdges();
        for (auto edge : outgoingEdges)
        {
            if (edgeFilter(edge))
            {
                Node *toNode = edge->getDest();
                // double alt = node->getDistance() + edgeWeight(edge);
                double alt = node->getNodeDistance() - edgeWeight(edge);
                if (alt < toNode->getNodeDistance())
                {
                    // toNode->setDistance(alt);
                    // queue.push(toNode);
                    pQ.set(toNode, alt);
                    toNode->setPreviousEdge(edge);
                }
            }
        }
    }
    path = buildPath(source, dest);

    return path;
}

vector<Edge *> Multigraph::dijkstraShortestPathEdgesByNode(Node *source, Node *dest, bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *))
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    better_priority_queue::updatable_priority_queue<Node *, double> pQ;

    // priority_queue<Node *, vector<Node *>, NodeDistanceComparator> queue;

    for (auto node : nodes)
    {
        node->resetNode();
    }
    // source->setDistance(0);
    // queue.push(source);
    pQ.set(source, 0);

    // thank you copilot <3
    while (!pQ.empty())
    {
        Node *node = pQ.pop_value().key;
        // check if node is destination
        // Node *node = queue.top();
        // queue.pop();
        // if(node->isFound()){
        //     continue;
        // }
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

            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    double weight = edgeWeight(edge);
                    if (weight < bestWeight)
                    {
                        bestWeight = weight;
                        bestEdge = edge;
                    }
                }
            }

            // use the best edge for dijkstra
            double alt = node->getNodeDistance() - edgeWeight(bestEdge);
            if (alt < toNode->getNodeDistance())
            {
                // toNode->setDistance(alt);
                // queue.push(toNode);
                pQ.set(toNode, alt);
                toNode->setPreviousEdge(bestEdge);
            }
        }
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
    return path;
}

vector<vector<Edge *>> Multigraph::getShortestPathDijkstra(
    vector<Node *> nodes,
    bool (*edgeFilter)(Edge *),
    double (*edgeWeight)(Edge *),
    vector<Edge *> (*dijkstra)(Node *, Node *, bool (*)(Edge *), double (*)(Edge *)))
{
    {
        if (nodes.size() < 2)
        {
            throw "Not enough nodes";
        }
        vector<vector<Edge *>> allPaths;
        for (auto i = 1; i < nodes.size(); i++)
        {
            vector<Edge *> path = dijkstra(nodes[i - 1], nodes[i], edgeFilter, edgeWeight);
            allPaths.push_back(path);
        }
        return allPaths;
    }
}

vector<Edge *> Multigraph::bfs(
    Node *n1,
    Node *n2,
    bool (*edgeFilter)(Edge *))
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
    bool (*edgeFilter)(Edge *))
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
    bool (*edgeFilter)(Edge *),
    vector<Edge *> (*bfs)(Node *, Node *, bool (*)(Edge *)))
{
    vector<Edge *> path = bfs(n1, n2, edgeFilter);
    return make_pair(path, path.size());
}

vector<Edge *> Multigraph::dfs(
    Node *n1,
    Node *n2,
    bool (*edgeFilter)(Edge *))
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
    bool (*edgeFilter)(Edge *))
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
    bool (*edgeFilter)(Edge *))
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
    bool (*edgeFilter)(Edge *))
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

vector<Edge *> Multigraph::getEdges(bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *))
{
    vector<Edge *> filteredEdges;
    copy_if(this->edges.begin(), this->edges.end(), filteredEdges.begin(), edgeFilter);
    return filteredEdges;
}

vector<Edge *> Multigraph::getBestEdges(bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *))
{
    // I think this is useles. Will only get the edges with least weight for the local minimum spanning tree
    // We need every edge
    vector<Edge *> bestEdges;
    double bestWeight = numeric_limits<double>::max(); // Initialize to a large number
    for (auto edge : this->edges)
    {
        if (edgeFilter(edge))
        {
            double weight = edgeWeight(edge);
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

vector<Edge *> Multigraph::getBestEdgesByNode(Node *node, bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *))
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
                double weight = edgeWeight(edge);
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

bool Multigraph::isConnected(Node *n1, bool (*edgeFilter)(Edge *), vector<Edge *> (*dfs)(Node *, bool (*edgeFilter)(Edge *)))
{
    dfs(n1, edgeFilter); 

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
    bool (*edgeFilter)(Edge *),
    double (*edgeWeight)(Edge *),
    vector<Edge *> (*collectEdges)(bool (*edgeFilter)(Edge *), double (*edgeWeight)(Edge *)),
    vector<Edge *> (*dfs)(Node *, bool (*)(Edge *)))
{

    vector<Edge *> edges = collectEdges(edgeFilter, edgeWeight);
    vector<Edge *> mstEdges = vector<Edge *>();

    // sort edges by weight
    sort(edges.begin(), edges.end(), [edgeWeight](Edge *e1, Edge *e2)
         { edgeWeight(e1) < edgeWeight(e2); });

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
                return it!=e->getSource()->getAuxOutgoingEdges().end(); 
                },
            dfs);

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
