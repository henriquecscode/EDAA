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
    string stateName;

public:
    Airport(double lon, double lat, int id, string code, string name, string city, string state, string statename);
    Airport(double lon, double lat);

    double getLongitude();
    double getLatitude();
    int getId();
    string getCode();
    string getName();
    string getCity();
    string getState();
    string getStateName();

    string toString();
};

#endif