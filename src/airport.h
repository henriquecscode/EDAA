#ifndef AIRPORT_H
#define AIRPORT_H

using namespace std;
#include <string>

class Airport
{
    double longitude;
    double latitude;
    int id;
    string code;
    string name;
    string city;
    string state;
    string statename;

public:
    Airport(double lon, double lat, int id, string code, string name, string city, string state, string statename);
    Airport(double lon, double lat);
};

#endif