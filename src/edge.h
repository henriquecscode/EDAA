#ifndef EDGE_H
#define EDGE_H

#include <string>
#include <functional>
#include "flight.h"

using namespace std;

class Node;


class Edge
{
    Node *source;
    Node *dest;
    Flight data;

public:
    Edge(Node *source, Node *dest, Flight data);
    Node *getSource();
    Node *getDest();
    Flight *getData();

    // function<bool (Edge*)>getEdgeFilter(double (*f)(Flight*), double min, double max);

    static function<bool(Edge *)> getEdgeFilter();
    // static function<bool(Edge *)> getEdgeFilter(string dataAttributeName, double min, double max);
    static function<bool(Edge *)> getEdgeFilter(function<double (Flight*)> f, double min, double max);
    static function<bool(Edge *)> getEdgeFilter(function<int (Flight*)> f, int min, int max);
    static function<bool(Edge *)> getEdgeFilter(function<string (Flight*)> f, string comparison);

    static function<double(Edge *)> getEdgeWeight(string dataAttributeName);
    static function<double(Edge *)> getDoubleEdgeWeight(function<double (Flight*)> f);
    static function<double(Edge *)> getIntEdgeWeight(function<int (Flight*f)> f);
};
#endif
