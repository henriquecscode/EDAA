#include "multigraph.h"
#include "node.h"
#include <vector>
#include <queue>
#include <set>
#include <utility>
#include "better_priority_queue.h"
using namespace std;

template <typename NodeT, typename EdgeT>
Multigraph<NodeT, EdgeT>::Multigraph()
{
    nodes = vector<Node<NodeT> *>();
    edges = vector<Edge<EdgeT> *>();
};

template <typename NodeT, typename EdgeT>
bool Multigraph<NodeT, EdgeT>::createNode(NodeT data)
{
    Node<NodeT> *node = new Node<NodeT>(data);
    this->nodes.push_back(node);
    return true;
}

template <typename NodeT, typename EdgeT>
bool Multigraph<NodeT, EdgeT>::createEdge(Node<NodeT> *source, Node<NodeT> *dest, EdgeT data)
{
    Edge<EdgeT> *edge = new Edge<EdgeT>(source, dest, data);
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
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::dijkstraShortestPath(Node<NodeT> *source, Node<NodeT> *dest, bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *))
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    better_priority_queue::updatable_priority_queue<Node<NodeT> *, double> pQ;

    // priority_queue<Node<NodeT> *, vector<Node<NodeT> *>, NodeDistanceComparator> queue;

    for (auto node : nodes)
    {
        node->resetNode();
    }
    // source->setDistance(0);
    // queue.push(source);
    pq.set(source, 0);

    // thank you copilot <3 <3
    while (!queue.empty())
    {
        Node<NodeT> *node = queue.top();
        queue.pop();
        // check if node is destination
        if (node == dest)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT> *toNode = edge->getDest();
                // double alt = node->getDistance() + edgeWeight(edge);
                double alt = node->getDistance() - edgeWeight(edge);
                if (alt < toNode->getDistance())
                {
                    // toNode->setDistance(alt);
                    // queue.push(toNode);
                    pq.set(toNode, alt);
                    toNode->setPrevious(node);
                }
            }
        }
    }
    vector<Edge<EdgeT> *> path = buildPath(source, dest);

    return path;
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::dijkstraShortestPathEdgesByNode(Node<NodeT> *source, Node<NodeT> *dest, bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *))
{
    // pQ is a maximum priority queue. We want to use a minimum priority queue, so we negate the distance.
    better_priority_queue::updatable_priority_queue<Node<NodeT> *, double> pQ;

    // priority_queue<Node<NodeT> *, vector<Node<NodeT> *>, NodeDistanceComparator> queue;

    for (auto node : nodes)
    {
        node->resetNode();
    }
    // source->setDistance(0);
    // queue.push(source);
    pq.set(source, 0);

    // thank you copilot <3
    while (!queue.empty())
    {
        Node<NodeT> *node = queue.top();
        queue.pop();
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
            Edge<EdgeT> *bestEdge = nullptr;
            Node<NodeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;

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
                pq.set(toNode, alt);
                toNode->setPrevious(node);
            }
        }
    }
    vector<Edge<EdgeT> *> path = buildPath(source, dest);
    return path;
}

// build path
template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::buildPath(Node<NodeT> *source, Node<NodeT> *dest)
{
    vector<Edge<EdgeT> *> path;
    Node<NodeT> *node = dest;

    while (node != source)
    {
        path.push_back(node->getPreviousEdge());
        node = node->getPreviousEdge()->getSource();
    }
    return path;
}

template <typename NodeT, typename EdgeT>
vector<vector<Edge<EdgeT> *>> Multigraph<NodeT, EdgeT>::getShortestPathDijkstra(
    vector<Node<NodeT> *> nodes,
    bool (*edgeFilter)(Edge<EdgeT> *),
    double (*edgeWeight)(Edge<EdgeT> *),
    vector<Edge<EdgeT> *> (*dijkstra)(Node<NodeT> *, Node<NodeT> *, bool (*)(Edge<EdgeT> *), double (*)(Edge<EdgeT> *)))
{
    {
        if (nodes.length < 2)
        {
            throw "Not enough nodes";
        }
        vector<vector<Edge<EdgeT> *>> allPaths;
        for (auto i = 1; i < nodes.length; i++)
        {
            vector<Edge<EdgeT> *> path = dijkstra(nodes[i - 1], nodes[i], edgeFilter, edgeWeight);
            allPaths.push_back(path);
        }
        return allPaths;
    }
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::bfs(
    Node<NodeT> *n1,
    Node<NodeT> *n2,
    bool (*edgeFilter)(Edge<EdgeT> *))
{
    queue<Node<NodeT> *> q;
    vector<Edge<EdgeT> *> path;

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
        Node<NodeT> *node = q.front();
        q.pop();
        if (node == n2)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT> *toNode = edge->getDest();
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

    vector<Edge<EdgeT> *> path = buildPath(n1, n2);
    return path;
}
template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::bfsByNode(
    Node<NodeT> *n1,
    Node<NodeT> *n2,
    bool (*edgeFilter)(Edge<EdgeT> *))
{
    queue<Node<NodeT> *> q;
    vector<Edge<EdgeT> *> path;

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
        Node<NodeT> *node = q.front();
        q.pop();
        if (node == n2)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node<NodeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
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

    vector<Edge<EdgeT> *> path = buildPath(n1, n2);
    return path;
}

template <typename NodeT, typename EdgeT>
pair<vector<Edge<EdgeT> *>, int> Multigraph<NodeT, EdgeT>::getErdos(
    Node<NodeT> *n1,
    Node<NodeT> *n2,
    bool (*edgeFilter)(Edge<EdgeT> *),
    vector<Edge<EdgeT> *> (*bfs)(Node<NodeT> *, Node<NodeT> *, bool (*)(Edge<EdgeT> *)))
{
    vector<Edge<EdgeT> *> path = bfs(n1, n2, edgeFilter);
    return make_pair(path, path.size());
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::dfs(
    Node<NodeT> *n1,
    Node<NodeT> *n2,
    bool (*edgeFilter)(Edge<EdgeT> *))
{
    stack<Node<NodeT> *> s;
    vector<Edge<EdgeT> *> path;

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
        Node<NodeT> *node = s.top();
        s.pop();
        if (node == n2)
        {
            break;
        }

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT> *toNode = edge->getDest();
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

    vector<Edge<EdgeT> *> path = buildPath(n1, n2);
    return path;
}

template <typename NodeT, typename EdgeT>
map<Node<NodeT> *, vector<Edge<EdgeT> *>> Multigraph<NodeT, EdgeT>::dfs(
    Node<NodeT> *n1,
    bool (*edgeFilter)(Edge<EdgeT> *))
{
    set<Node<NodeT> *> found;
    stack<Node<NodeT> *> s;
    vector<Edge<EdgeT> *> path;

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node<NodeT> *node = s.top();
        s.pop();

        for (auto edge : node->getOutgoingEdges())
        {
            if (edgeFilter(edge))
            {
                Node<NodeT> *toNode = edge->getDest();
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

    map<Node<NodeT> *, vector<Edge<EdgeT> *>> pathMap;
    for (auto node : nodes)
    {
        if (found.find(node) != found.end())
        {
            vector<Edge<EdgeT> *> nodePath = buildPath(n1, node);
            pathMap[node] = nodePath;
        }
    }
    return pathMap;
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::dfsByNode(
    Node<NodeT> *n1,
    Node<NodeT> *n2,
    bool (*edgeFilter)(Edge<EdgeT> *))
{
    stack<Node<NodeT> *> s;
    vector<Edge<EdgeT> *> path;

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
        Node<NodeT> *node = s.top();
        s.pop();
        if (node == n2)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node<NodeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
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

    vector<Edge<EdgeT> *> path = buildPath(n1, n2);
    return path;
}

template <typename NodeT, typename EdgeT>
map<Node<NodeT> *, vector<Edge<EdgeT> *>> Multigraph<NodeT, EdgeT>::dfsByNode(
    Node<NodeT> *n1,
    bool (*edgeFilter)(Edge<EdgeT> *))
{
    set<Node<NodeT> *> found;
    stack<Node<NodeT> *> s;

    for (auto node : nodes)
    {
        node->resetNode();
    }

    // dfs over nodes using visited
    s.push(n1);
    n1->find();
    while (!s.empty())
    {
        Node<NodeT> *node = s.top();
        s.pop();

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for(auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            Node<NodeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
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

    map<Node<NodeT> *, vector<Edge<EdgeT> *>> pathMap;
    for (auto node : found)
    {
        vector<Edge<EdgeT> *> nodePath = buildPath(n1, node);
        pathMap[node] = nodePath;
    }
    return pathMap;
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> Multigraph<NodeT, EdgeT>::getEdges(bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *))
{
    // check if this does a copy or not
    // It should
    return getEdges();
}

template <typename NodeT, typename EdgeT>
vector<Edge<EdgeT> *> getBestEdges(bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *))
{
    vector<Edge<EdgeT> *> bestEdges;
    double bestWeight = numeric_limits<double>::max(); // Initialize to a large number
    for (auto edge : edges)
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
vector<Edge<EdgeT> *> getBestEdgesByNode(Node<NodeT> *node, bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *))
{
    vector<Edge<EdgeT> *> bestEdges;
    double bestWeight = numeric_limits<double>::max(); // Initialize to a large number
    auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
    for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
    {
        Node<NodeT> *toNode = outgoingEdgesOfNode.first;
        vector<Edge<EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
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
bool Multigraph<NodeT, EdgeT>::isConnected(Node<NodeT> *n1, bool (*edgeFilter)(Edge<EdgeT> *), vector<Edge<EdgeT> *> (*dfs)(Node<NodeT> *))
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
void Multigraph<NodeT, EdgeT>::mountTree(Node<NodeT> *root, vector<Edge<EdgeT> *> treeEdges)
{
    return;
}

template <typename NodeT, typename EdgeT>
void Multigraph<NodeT, EdgeT>::getLocalMinimumSpanningTree(
    Node<NodeT> *localNode,
    bool (*edgeFilter)(Edge<EdgeT> *),
    double (*edgeWeight)(Edge<EdgeT> *),
    vector<Edge<EdgeT> *> (*collectEdges)(bool (*edgeFilter)(Edge<EdgeT> *), double (*edgeWeight)(Edge<EdgeT> *)),
    vector<Edge<EdgeT> *> (*dfs)(Node<NodeT> *, Node<NodeT> *, bool (*)(Edge<EdgeT> *)))
{

    vector<Edge<EdgeT *>> edges = collectEdges(edgeFilter, edgeWeight);
    vector<Edge<EgdeT> *> mstEdges = new vector<Edge<EdgeT> *>();

    // sort edges by weight
    sort(edges.begin(), edges.end(), [](Edge<EdgeT> *e1, Edge<EdgeT> *e2)
         { edgeWeight(e1) < edgeWeight(e2); });

    for (auto node : nodes)
    {
        node->resetNode();
    }

    for (auto edge : edges)
    {
        Node<NodeT> *n1 = edge->getSource();
        Node<NodeT> *n2 = edge->getDest();
        n1->addAuxOutgoingEdge(edge);
        n2->addAuxIncomingEdge(edge);
    }

    for (auto edge : edges)
    {
        Node<NodeT> *n1 = edge->getSource();
        Node<NodeT> *n2 = edge->getDest();

        n1->removeAuxOutgoingEdge(edge);
        n2->removeAuxIncomingEdge(edge);

        bool connected = isConnected(
            localNode, [](Edge<EdgeT> *e)
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
