#include "node.h"
#include <vector>

using namespace std;

template <class T>
class Node
{
private:
    T data;
    vector<Edge *> incomingEdges;
    vector<Edge *> outgoingEdges;

public:
    Node(T data);

    T &getData()
    {
        return data;
    }

    addIncomingEdge(Edge *edge)
    {
        incomingEdges.push_back(edge);
    }

    addOutgoingEdge(Edge *edge)
    {
        outgoingEdges.push_back(edge);
    }

    addEdge(Edge *edge)
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
    }
};