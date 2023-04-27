#ifndef ATTRIBUTE_TYPE_H
#define ATTRIBUTE_TYPE_H

#include <functional>
class Edge;

enum attributeType
{
    INT,
    DOUBLE,
    STRING
};

// typedef function<double(Edge *, double&)> EdgeWeight;
using EdgeWeighter = std::function<double(Edge *, double &)>;
using EdgeFilter = std::function<bool(Edge *)>;
#endif