#include "flight.h"
using namespace std;

Flight::Flight(int dayMonth, int dayWeek, string carrier, int originId, int destId, int depDelay, int arrDelay, double distance, double flightTime)
{
    this->dayMonth = dayMonth;
    this->dayWeek = dayWeek;
    this->carrier = carrier;
    this->originId = originId;
    this->destId = destId;
    this->depDelay = depDelay;
    this->arrDelay = arrDelay;
    this->distance = distance;
    this->flightTime = flightTime;
}
Flight::Flight(double distance, double flightTime)
{
    this->distance = distance;
    this->flightTime = flightTime;
}