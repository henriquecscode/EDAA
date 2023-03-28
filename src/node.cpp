#include "node.h"
#include <vector>

using namespace std;

template <class T>
Node<T>::Node(T data)
{
    this->data = data;
}

template <class T>
T &Node<T>::getData()
{
    return data;
}

template <class T>
bool Node<T>::addIncomingEdge(Edge *edge)
{
    incomingEdges.push_back(edge);
    return true;
}

template <class T>
bool Node<T>::addOutgoingEdge(Edge *edge)
{
    outgoingEdges.push_back(edge);
    return true;
}

template <class T>
bool Node<T>::addEdge(Edge *edge)
{
    if (edge->getSource() == this)
    {
        addOutgoingEdge(edge);
        edge->getDest()->addIncomingEdge(edge);
    }
    else if (edge->getDest() == this)
    {
        edge->getSource()
            ->addOutgoingEdge(edge);
        addIncomingEdge(edge);
    }
    else
    {
        throw "Edge does not connect to this node";
    }
    return true;
};