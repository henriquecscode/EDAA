#ifndef FLIGHT_H
#define FLIGHT_H

#include <string>
using namespace std;

class Flight
{
    int dayMonth;
    int dayWeek;
    string carrier;
    int originId;
    int destId;
    int depDelay;
    int arrDelay;
    double distance;
    double flightTime;

public:
    Flight(int dayMonth, int dayWeek, string carrier, int originId, int destId, int depDelay, int arrDelay, double distance, double flightTime);
    Flight(double distance, double flightTime);
};

#endif