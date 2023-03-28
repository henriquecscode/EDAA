#include "airport.h"
using namespace std;

Airport::Airport(double lon, double lat, int id, string code, string name, string city, string state, string statename)
{
    this->longitude = lon;
    this->latitude = lat;
    this->id = id;
    this->code = code;
    this->name = name;
    this->city = city;
    this->state = state;
    this->statename = statename;
}

Airport::Airport(double lon, double lat)
{
    this->longitude = lon;
    this->latitude = lat;
}
