#include "multigraph.h"
#include "node.h"
#include <vector>
#include <queue>
#include <set>
#include <utility>
#include <limits>
#include <algorithm>
#include <stack>
#include "better_priority_queue.h"
using namespace std;

template <typename NodeT, typename EdgeT>
Multigraph<NodeT, EdgeT>::Multigraph()
{
    nodes = vector<Node<NodeT, EdgeT> *>();
    edges = vector<Edge<NodeT, EdgeT> *>();
};

template <typename NodeT, typename EdgeT>
Node<NodeT, EdgeT> Multigraph<NodeT, EdgeT>::createNode(NodeT data)
{
    Node<NodeT, EdgeT> *node = new Node<NodeT, EdgeT>(data);
    this->nodes.push_back(node);
    return node;
}

template <typename NodeT, typename EdgeT>
bool Multigraph<NodeT, EdgeT>::createEdge(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, EdgeT data)
{
    Edge<NodeT, EdgeT> *edge = new Edge<NodeT, EdgeT>(source, dest, data);
    this->edges.push_back(edge);
    source->addEdge(edge);
    return true;
}

template <typename NodeT, typename EdgeT>
void *Multigraph<NodeT, EdgeT>::getNodes()
{
    return this->nodes;
}

template <typename NodeT, typename EdgeT>
void *Multigraph<NodeT, EdgeT>::getEdges()
{
    return this->edges;
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::dijkstraShortestPath(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *))
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    better_priority_queue::updatable_priority_queue<Node<NodeT, EdgeT> *, double> pQ;
    vector<Edge<NodeT, EdgeT> *> path;
    // priority_queue<Node<NodeT, EdgeT> *, vector<Node<NodeT, EdgeT> *>, NodeDistanceComparator> queue;

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
        Node<NodeT, EdgeT> *node = pQ.top();
        pQ.pop();
        // check if node is destination
        if (node == dest)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT, EdgeT> *toNode = edge->getDest();
                // double alt = node->getDistance() + edgeWeight(edge);
                double alt = node->getDistance() - edgeWeight(edge);
                if (alt < toNode->getDistance())
                {
                    // toNode->setDistance(alt);
                    // queue.push(toNode);
                    pQ.set(toNode, alt);
                    toNode->setPrevious(node);
                }
            }
        }
    }
    path = buildPath(source, dest);

    return path;
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::dijkstraShortestPathEdgesByNode(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *))
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    better_priority_queue::updatable_priority_queue<Node<NodeT, EdgeT> *, double> pQ;

    // priority_queue<Node<NodeT, EdgeT> *, vector<Node<NodeT, EdgeT> *>, NodeDistanceComparator> queue;

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
        Node<NodeT, EdgeT> *node = pQ.top();
        pQ.pop();
        // check if node is destination
        if (node == dest)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            // get the best edge
            double bestWeight = numeric_limits<double>::max();
            Edge<NodeT, EdgeT> *bestEdge = nullptr;
            Node<NodeT, EdgeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<NodeT, EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;

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
            double alt = node->getDistance() - edgeWeight(bestEdge);
            if (alt < toNode->getDistance())
            {
                // toNode->setDistance(alt);
                // queue.push(toNode);
                pQ.set(toNode, alt);
                toNode->setPrevious(node);
            }
        }
    }
    vector<Edge<NodeT, EdgeT> *> path = buildPath(source, dest);
    return path;
}

// build path
template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::buildPath(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest)
{
    vector<Edge<NodeT, EdgeT> *> path;
    Node<NodeT, EdgeT> *node = dest;

    while (node != source)
    {
        path.push_back(node->getPreviousEdge());
        node = node->getPreviousEdge()->getSource();
    }
    return path;
}

template <typename NodeT, typename EdgeT>
vector<vector<Edge<NodeT, EdgeT> *>> Multigraph<NodeT, EdgeT>::getShortestPathDijkstra(
    vector<Node<NodeT, EdgeT> *> nodes,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *),
    double (*edgeWeight)(Edge<NodeT, EdgeT> *),
    vector<Edge<NodeT, EdgeT> *> (*dijkstra)(Node<NodeT, EdgeT> *, Node<NodeT, EdgeT> *, bool (*)(Edge<NodeT, EdgeT> *), double (*)(Edge<NodeT, EdgeT> *)))
{
    {
        if (nodes.length < 2)
        {
            throw "Not enough nodes";
        }
        vector<vector<Edge<NodeT, EdgeT> *>> allPaths;
        for (auto i = 1; i < nodes.length; i++)
        {
            vector<Edge<NodeT, EdgeT> *> path = dijkstra(nodes[i - 1], nodes[i], edgeFilter, edgeWeight);
            allPaths.push_back(path);
        }
        return allPaths;
    }
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::bfs(
    Node<NodeT, EdgeT> *n1,
    Node<NodeT, EdgeT> *n2,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *))
{
    queue<Node<NodeT, EdgeT> *> q;
    vector<Edge<NodeT, EdgeT> *> path;

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
        Node<NodeT, EdgeT> *node = q.front();
        q.pop();
        if (node == n2)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT, EdgeT> *toNode = edge->getDest();
                if (!toNode->isFound())
                {
                    // Not found
                    q.push(toNode);
                    toNode->setPrevious(node);
                    toNode->find();
                }
            }
        }
    }

    path = buildPath(n1, n2);
    return path;
}
template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::bfsByNode(
    Node<NodeT, EdgeT> *n1,
    Node<NodeT, EdgeT> *n2,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *))
{
    queue<Node<NodeT, EdgeT> *> q;
    vector<Edge<NodeT, EdgeT> *> path;

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
        Node<NodeT, EdgeT> *node = q.front();
        q.pop();
        if (node == n2)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node<NodeT, EdgeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<NodeT, EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    if (!toNode->isFound())
                    {
                        // Not found
                        q.push(toNode);
                        toNode->setPrevious(node);
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

template <typename NodeT, typename EdgeT>
pair<vector<Edge<NodeT, EdgeT> *>, int> Multigraph<NodeT, EdgeT>::getErdos(
    Node<NodeT, EdgeT> *n1,
    Node<NodeT, EdgeT> *n2,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *),
    vector<Edge<NodeT, EdgeT> *> (*bfs)(Node<NodeT, EdgeT> *, Node<NodeT, EdgeT> *, bool (*)(Edge<NodeT, EdgeT> *)))
{
    vector<Edge<NodeT, EdgeT> *> path = bfs(n1, n2, edgeFilter);
    return make_pair(path, path.size());
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::dfs(
    Node<NodeT, EdgeT> *n1,
    Node<NodeT, EdgeT> *n2,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *))
{
    stack<Node<NodeT, EdgeT> *> s;
    vector<Edge<NodeT, EdgeT> *> path;

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
        Node<NodeT, EdgeT> *node = s.top();
        s.pop();
        if (node == n2)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT, EdgeT> *toNode = edge->getDest();
                if (!toNode->isFound())
                {
                    // Not found
                    s.push(toNode);
                    toNode->setPrevious(node);
                    toNode->find();
                }
            }
        }
    }

    path = buildPath(n1, n2);
    return path;
}

template <typename NodeT, typename EdgeT>
map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> Multigraph<NodeT, EdgeT>::dfs(
    Node<NodeT, EdgeT> *n1,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *))
{
    set<Node<NodeT, EdgeT> *> found;
    stack<Node<NodeT, EdgeT> *> s;
    vector<Edge<NodeT, EdgeT> *> path;

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node<NodeT, EdgeT> *node = s.top();
        s.pop();

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT, EdgeT> *toNode = edge->getDest();
                if (found.find(toNode) == found.end())
                {
                    // Not found
                    s.push(toNode);
                    toNode->setPrevious(node);
                    found.insert(toNode);
                }
            }
        }
    }

    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> pathMap;
    for (auto node : nodes)
    {
        if (found.find(node) != found.end())
        {
            vector<Edge<NodeT, EdgeT> *> nodePath = buildPath(n1, node);
            pathMap[node] = nodePath;
        }
    }
    return pathMap;
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::dfsByNode(
    Node<NodeT, EdgeT> *n1,
    Node<NodeT, EdgeT> *n2,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *))
{
    stack<Node<NodeT, EdgeT> *> s;
    vector<Edge<NodeT, EdgeT> *> path;

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
        Node<NodeT, EdgeT> *node = s.top();
        s.pop();
        if (node == n2)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node<NodeT, EdgeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<NodeT, EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    if (!toNode->isFound())
                    {
                        // Not found
                        s.push(toNode);
                        toNode->setPrevious(node);
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

template <typename NodeT, typename EdgeT>
map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> Multigraph<NodeT, EdgeT>::dfsByNode(
    Node<NodeT, EdgeT> *n1,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *))
{
    set<Node<NodeT, EdgeT> *> found;
    stack<Node<NodeT, EdgeT> *> s;

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node<NodeT, EdgeT> *node = s.top();
        s.pop();

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node<NodeT, EdgeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<NodeT, EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    if (!toNode->isFound())
                    {
                        // Not found
                        s.push(toNode);
                        toNode->setPrevious(node);
                        toNode->find();
                    }
                }
            }
        }
    }

    map<Node<NodeT, EdgeT> *, vector<Edge<NodeT, EdgeT> *>> pathMap;
    for (auto node : found)
    {
        vector<Edge<NodeT, EdgeT> *> nodePath = buildPath(n1, node);
        pathMap[node] = nodePath;
    }
    return pathMap;
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::getEdges(bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *))
{
    vector<Edge<NodeT, EdgeT> *> filteredEdges;
    copy_if(this.edges.begin(), this.edges.end(), filteredEdges.begin(), edgeFilter);
    return filteredEdges;
}

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::getBestEdges(bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *))
{
    // I think this is useles. Will only get the edges with least weight for the local minimum spanning tree
    // We need every edge
    vector<Edge<NodeT, EdgeT> *> bestEdges;
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

template <typename NodeT, typename EdgeT>
vector<Edge<NodeT, EdgeT> *> Multigraph<NodeT, EdgeT>::getBestEdgesByNode(Node<NodeT, EdgeT> *node, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *))
{
    vector<Edge<NodeT, EdgeT> *> bestEdges;
    double bestWeight = numeric_limits<double>::max(); // Initialize to a large number
    auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
    for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
    {
        Node<NodeT, EdgeT> *toNode = outgoingEdgesOfNode.first;
        vector<Edge<NodeT, EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
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

template <typename NodeT, typename EdgeT>
bool Multigraph<NodeT, EdgeT>::isConnected(Node<NodeT, EdgeT> *n1, bool (*edgeFilter)(Edge<NodeT, EdgeT> *), vector<Edge<NodeT, EdgeT> *> (*dfs)(Node<NodeT, EdgeT> *))
{
    dfs(n1, edgeFilter);

    for (auto node : nodes)
    {
        if (!node->isVisited())
        {
            return false;
        }
    }
    return true;
}

template <typename NodeT, typename EdgeT>
void Multigraph<NodeT, EdgeT>::mountTree(Node<NodeT, EdgeT> *root, vector<Edge<NodeT, EdgeT> *> treeEdges)
{
    return;
}

template <typename NodeT, typename EdgeT>
void Multigraph<NodeT, EdgeT>::getLocalMinimumSpanningTree(
    Node<NodeT, EdgeT> *localNode,
    bool (*edgeFilter)(Edge<NodeT, EdgeT> *),
    double (*edgeWeight)(Edge<NodeT, EdgeT> *),
    vector<Edge<NodeT, EdgeT> *> (*collectEdges)(bool (*edgeFilter)(Edge<NodeT, EdgeT> *), double (*edgeWeight)(Edge<NodeT, EdgeT> *)),
    vector<Edge<NodeT, EdgeT> *> (*dfs)(Node<NodeT, EdgeT> *, Node<NodeT, EdgeT> *, bool (*)(Edge<NodeT, EdgeT> *)))
{

    vector<Edge<NodeT, EdgeT> *> edges = collectEdges(edgeFilter, edgeWeight);
    vector<Edge<NodeT, EdgeT> *> mstEdges = new vector<Edge<NodeT, EdgeT> *>();

    // sort edges by weight
    sort(edges.begin(), edges.end(), [edgeWeight](Edge<NodeT, EdgeT> *e1, Edge<NodeT, EdgeT> *e2)
         { edgeWeight(e1) < edgeWeight(e2); });

    for (auto node : nodes)
    {
        node->resetNode();
    }

    for (auto edge : edges)
    {
        Node<NodeT, EdgeT> *n1 = edge->getSource();
        Node<NodeT, EdgeT> *n2 = edge->getDest();
        n1->addAuxOutgoingEdge(edge);
        n2->addAuxIncomingEdge(edge);
    }

    for (auto edge : edges)
    {
        Node<NodeT, EdgeT> *n1 = edge->getSource();
        Node<NodeT, EdgeT> *n2 = edge->getDest();

        n1->removeAuxOutgoingEdge(edge);
        n2->removeAuxIncomingEdge(edge);

        bool connected = isConnected(
            localNode, [](Edge<NodeT, EdgeT> *e)
            { return e->getSource()->getAuxOutgoingEdges().contains(e); },
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
