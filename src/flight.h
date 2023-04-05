#ifndef FLIGHT_H
#define FLIGHT_H

#include <string>
#include <functional>
using namespace std;

enum attributeType {INT, DOUBLE, STRING};
union valueType{
    int intValue;
    double doubleValue;
    string stringValue;
    // value() {memset(this, 0, sizeof(*this));}
};

struct{
    attributeType type;
    valueType value;
    // Filter() {memset(this, 0, sizeof(*this));}
} typedef Filter;

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

    int getDayMonth();
    int getDayWeek();
    string getCarrier();
    int getOriginId();
    int getDestId();
    int getDepDelay();
    int getArrDelay();
    double getDistance();
    double getFlightTime();

    // void *getGetter(string attribute);
    // static std::function<void *(Flight *)> getGetter(string attribute);
    static std::function<int(Flight *)> getIntGetter(string attribute);
    static std::function<double(Flight *)> getDoubleGetter(string attribute);
    static std::function<string(Flight *)> getStringGetter(string attribute);
};

#endif