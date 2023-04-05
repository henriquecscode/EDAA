#include "edge.h"
#include <string>
#include <functional>
using namespace std;

template <typename NodeT, typename EdgeT>
Edge<NodeT, EdgeT>::Edge(Node<NodeT, EdgeT> *source, Node<NodeT, EdgeT> *dest, EdgeT data)
{
    this->source = source;
    this->dest = dest;
    this->data = data;
}

template <typename NodeT, typename EdgeT>
Node<NodeT, EdgeT> *Edge<NodeT, EdgeT>::getSource()
{
    return source;
}

template <typename NodeT, typename EdgeT>
Node<NodeT, EdgeT> *Edge<NodeT, EdgeT>::getDest()
{
    return dest;
}

template <typename NodeT, typename EdgeT>
EdgeT *Edge<NodeT, EdgeT>::getData()
{
    return &data;
}

template <typename NodeT, typename EdgeT>
function<bool(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeFilter(double (EdgeT::*f)(), double min, double max)
{
    return [f, min, max](Edge<NodeT, EdgeT> *edge) -> bool
    {
        EdgeT *data = edge->getData();
        double value = (*data.*f)();
        return min <= value && value <= max;
    };
}

template <typename NodeT, typename EdgeT>
function<bool(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeFilter(int (EdgeT::*f)(), int min, int max)
{
    return [f, min, max](Edge<NodeT, EdgeT> *edge) -> bool
    {
        EdgeT *data = edge->getData();
        int value = (*data.*f)();
        return min <= value && value <= max;
    };
}

template <typename NodeT, typename EdgeT>
function<bool(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeFilter(string (EdgeT::*f)(), string comparison)
{
    return [f, comparison](Edge<NodeT, EdgeT> *edge) -> bool
    {
        EdgeT *data = edge->getData();
        string value = (*data.*f)();
        return value == comparison;
    };
}

template <typename NodeT, typename EdgeT>
function<double(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeWeight(double (EdgeT::*f)())
{
    return [f](Edge<NodeT, EdgeT> *edge) -> double
    {
        EdgeT *data = edge->getData();
        double value = (*data.*f)();
        return value;
    };
}

template <typename NodeT, typename EdgeT>
function<double(Edge<NodeT, EdgeT> *)> Edge<NodeT, EdgeT>::getEdgeWeight(int (EdgeT::*f)())
{
    return [f](Edge<NodeT, EdgeT> *edge) -> double
    {
        EdgeT *data = edge->getData();
        int value = (*data.*f)();
        return static_cast<double>(value);
    };
}

// // 1 define a function pointer and initialize to NULL

// int (TMyClass::*pt2ConstMember)(float, char, char) const = NULL;

// // C++

// class TMyClass
// {
// public:
//    int DoIt(float a, char b, char c){ cout << "TMyClass::DoIt"<< endl; return a+b+c;};
//    int DoMore(float a, char b, char c) const
//          { cout << "TMyClass::DoMore" << endl; return a-b+c; };

//    /* more of TMyClass */
// };
// pt2ConstMember = &TMyClass::DoIt; // note: <pt2Member> may also legally point to &DoMore

// // Calling Function using Function Pointer

// (*this.*pt2ConstMember)(12, 'a', 'b');