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
    this->stateName = stateName;
}

Airport::Airport(double lon, double lat)
{
    this->longitude = lon;
    this->latitude = lat;
}

double Airport::getLongitude()
{
    return this->longitude;
}

double Airport::getLatitude()
{
    return this->latitude;
}

int Airport::getId()
{
    return this->id;
}

string Airport::getCode()
{
    return this->code;
}

string Airport::getName()
{
    return this->name;
}

string Airport::getCity()
{
    return this->city;
}

string Airport::getState()
{
    return this->state;
}

string Airport::getStateName()
{
    return this->stateName;
}