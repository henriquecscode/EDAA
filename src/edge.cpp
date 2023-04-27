#include "edge.h"
#include "flight.h"
#include <string>
#include <functional>
using namespace std;

Edge::Edge(Node *source, Node *dest, Flight data) : data(data)
{
    this->source = source;
    this->dest = dest;
    this->data = data;
}

Node *Edge::getSource()
{
    return source;
}

Node *Edge::getDest()
{
    return dest;
}

Flight *Edge::getData()
{
    return &data;
}

EdgeFilter Edge::getEdgeFilter()
{
    return [](Edge *edge) -> bool
    {
        return true;
    };
}

EdgeFilter Edge::getEdgeFilter(function<double(Flight *)> f, double min, double max)
{
    return [f, min, max](Edge *edge) -> bool
    {
        Flight *data = edge->getData();
        double value = f(data);
        return min <= value && value <= max;
    };
}

EdgeFilter Edge::getEdgeFilter(function<int(Flight *)> f, int min, int max)
{
    return [f, min, max](Edge *edge) -> bool
    {
        Flight *data = edge->getData();
        int value = f(data);
        return min <= value && value <= max;
    };
}

EdgeFilter Edge::getEdgeFilter(function<string(Flight *)> f, string comparison)
{
    return [f, comparison](Edge *edge) -> bool
    {
        Flight *data = edge->getData();
        string value = f(data);
        return value == comparison;
    };
}

EdgeWeighter Edge::getEdgeWeighter(string dataAttributeName)
{
    enum attributeType type = Flight::getAttributeType(dataAttributeName);
    if (type == INT)
    {
        auto func = Flight::getIntGetter(dataAttributeName);
        return Edge::getIntEdgeWeighter(func);
    }
    else if (type == DOUBLE)
    {
        auto func = Flight::getDoubleGetter(dataAttributeName);
        return Edge::getDoubleEdgeWeighter(func);
    }
    // switch (type)
    // {

    // case INT:
    //     auto func = Flight::getIntAttribute(dataAttributeName);
    //     return Edge::getEdgeWeight(func);

    //     {
    //         Flight *data = edge->getData();
    //         int value = (*data.*f)();
    //         return max(0, static_cast<double>(value)); // non-negative for dijsktra restrictions
    //     };
    // case DOUBLE:
    //     auto func = Flight::getDoubleAttribute(dataAttributeName) return [f](Edge * edge);
    //     return Edge::getEdgeWeight(func);
    //     // {
    //     //     Flight *data = edge->getData();
    //     //     double value = (*data.*f)();
    //     //     return max(0, value); // non-negative for dijsktra restrictions
    //     // };
    // }
}

EdgeWeighter Edge::getDoubleEdgeWeighter(function<double(Flight *)> f)
{
    return [f](Edge *edge, double &pointer) -> double
    {
        Flight *data = edge->getData();
        double value = f(data);
        double result = max(0.0, value);
        pointer = result; // non-negative for dijsktra restrictions
    };
}

EdgeWeighter Edge::getIntEdgeWeighter(function<int(Flight *f)> f)
{
    return [f](Edge *edge, double &pointer) -> double
    {
        Flight *data = edge->getData();
        int value = f(data);
        double result = max(0.0, static_cast<double>(value)); // non-negative for dijsktra restrictions
        pointer = result;
    };
}

string Edge::toString()
{
    return data.toString();
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