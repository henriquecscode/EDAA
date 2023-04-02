#include "multigraph.h"
#include "node.h"
#include <vector>
#include <queue>
#include "better_priority_queue.h"
using namespace std;

template <class NodeT, class EdgeT>
Multigraph<NodeT, EdgeT>::Multigraph()
{
    nodes = vector<Node<NodeT> *>();
    edges = vector<Edge<EdgeT> *>();
};

template <class NodeT, class EdgeT>
bool Multigraph<NodeT, EdgeT>::createNode(NodeT data)
{
    Node<NodeT> *node = new Node<NodeT>(data);
    this->nodes.push_back(node);
    return true;
}

template <class NodeT, class EdgeT>
bool Multigraph<NodeT, EdgeT>::createEdge(Node<NodeT> *source, Node<NodeT> *dest, EdgeT data)
{
    Edge<EdgeT> *edge = new Edge<EdgeT>(source, dest, data);
    this->edges.push_back(edge);
    source->addEdge(edge);
    return true;
}

template <class NodeT, class EdgeT>
void *Multigraph<NodeT, EdgeT>::getNodes()
{
    return &(this->nodes);
}

template <class NodeT, class EdgeT>
void *Multigraph<NodeT, EdgeT>::getEdges()
{
    return &(this->edges);
}

template <class NodeT, class EdgeT>
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

    // thank you copilot <3
    while (!queue.empty())
    {
        Node<NodeT> *node = queue.top();
        queue.pop();
        // check if node is destnation
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

template <class NodeT, class EdgeT>
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
        // check if node is destnation
        if (node == dest)
        {
            break;
        }

        auto outgoingEdgesByNode = node->getOutgoingEdgesByNode();
        for (auto outgoingEdgesOfNode : outgoingEdgesByNode)
        {
            //get the best edge
            Node<NodeT> *toNode = outgoingEdgesOfNode.first;
            vector<Edge<EdgeT> *> outgoingEdges = outgoingEdgesOfNode.second;
            priority_queue < Edge<EdgeT> *, vector<Edge<EdgeT> *>, class
            {
            public:
                bool operator()(Edge<EdgeT> *a, Edge<EdgeT> *b)
                {
                    return edgeWeight(a) < edgeWeight(b);
                }
            } > edgeQueue;
            for (auto edge : outgoingEdges)
            {
                if (edgeFilter(edge))
                {
                    queue.push(edge);
                }
            }

            //use the best edge for dijkstra
            Edge<EdgeT> *edge = edgeQueue.top();
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
    vector<Edge<EdgeT> *> path = buildPath(source, dest);
    return path;
}

// build path
template <class NodeT, class EdgeT>
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

template <class NodeT, class EdgeT>
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