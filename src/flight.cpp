#include "flight.h"
#include <string>
#include <functional>
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

int Flight::getDayMonth()
{
    return dayMonth;
}
int Flight::getDayWeek()
{
    return dayWeek;
}
string Flight::getCarrier()
{
    return carrier;
}

int Flight::getOriginId()
{
    return originId;
}
int Flight::getDestId()
{
    return destId;
}
int Flight::getDepDelay()
{
    return depDelay;
}
int Flight::getArrDelay()
{
    return arrDelay;
}
double Flight::getDistance()
{
    return distance;
}
double Flight::getFlightTime()
{
    return flightTime;
}

// std::function<void *(Flight *)> Flight::getGetter(string attribute)
// {
//     function<void *(Flight *)> func = nullptr;

//     if (attribute == "dayMonth")
//     {
//         func = [](Flight *f) -> void *
//         {
//             int dayMonth = f->getDayMonth();
//             Filter filter;
//             filter.type = INT;
//             filter.value.intValue = dayMonth;
//             return &filter;
//         };
//     }
//     else if (attribute == "dayWeek")
//     {
//         func = [](Flight *f) -> void *
//         {
//             int dayWeek = f->getDayWeek();
//             Filter filter;
//             filter.type = INT;
//             filter.value.intValue = dayWeek;
//             return &filter;
//         };
//     }
//     else if (attribute == "carrier")
//     {
//         func = [](Flight *f) -> void *
//         {
//             string carrier = f->getCarrier();
//             Filter filter;
//             filter.type = STRING;
//             filter.value.stringValue = carrier;
//             return &filter;
//         };
//     }
//     else if (attribute == "originId")
//     {
//         func = [](Flight *f) -> void *
//         {
//             int originId = f->getOriginId();
//             Filter filter;
//             filter.type = INT;
//             filter.value.intValue = originId;
//             return &filter;
//         };
//     }
//     else if (attribute == "destId")
//     {
//         func = [](Flight *f) -> void *
//         {
//             int destId = f->getDestId();
//             Filter filter;
//             filter.type = INT;
//             filter.value.intValue = destId;
//             return &filter;
//         };
//     }
//     else if (attribute == "depDelay")
//     {
//         func = [](Flight *f) -> void *
//         {
//             int depDelay = f->getDepDelay();
//             Filter filter;
//             filter.type = INT;
//             filter.value.intValue = depDelay;
//             return &filter;
//         };
//     }
//     else if (attribute == "arrDelay")
//     {
//         func = [](Flight *f) -> void *
//         {
//             int arrDelay = f->getArrDelay();
//             Filter filter;
//             filter.type = INT;
//             filter.value.intValue = arrDelay;
//             return &filter;
//         };
//     }
//     else if (attribute == "distance")
//     {
//         func = [](Flight *f) -> void *
//         {
//             double distance = f->getDistance();
//             Filter filter;
//             filter.type = DOUBLE;
//             filter.value.doubleValue = distance;
//             return &filter;
//         };
//     }
//     else if (attribute == "flightTime")
//     {
//         func = [](Flight *f) -> void *
//         {
//             double flightTime = f->getFlightTime();
//             Filter filter;
//             filter.type = DOUBLE;
//             filter.value.doubleValue = flightTime;
//             return &filter;
//         };
//     }
//     return func;
// }

attributeType Flight::getAttributeType(string attribute)
{
    if (attribute == "dayMonth" || attribute == "dayWeek" || attribute == "originId" || attribute == "destId" || attribute == "depDelay" || attribute == "arrDelay")
    {
        return INT;
    }
    else if (attribute == "distance" || attribute == "flightTime")
    {
        return DOUBLE;
    }
    else if (attribute == "carrier")
    {
        return STRING;
    }
}

std::function<int(Flight *)> Flight::getIntGetter(string attribute)
{
    function<int(Flight *)> func = nullptr;
    if (attribute == "dayMonth")
    {
        func = &Flight::getDayMonth;
    }
    else if (attribute == "dayWeek")
    {
        func = &Flight::getDayWeek;
    }
    else if (attribute == "originId")
    {
        func = &Flight::getOriginId;
    }
    else if (attribute == "destId")
    {
        func = &Flight::getDestId;
    }
    else if (attribute == "depDelay")
    {
        func = &Flight::getDepDelay;
    }
    else if (attribute == "arrDelay")
    {
        func = &Flight::getArrDelay;
    }
    return func;
}

std::function<double(Flight *)> Flight::getDoubleGetter(string attribute)
{
    function<double(Flight *)> func = nullptr;
    if (attribute == "distance")
    {
        func = &Flight::getDistance;
    }
    else if (attribute == "flightTime")
    {
        func = &Flight::getFlightTime;
    }
    return func;
}

std::function<string(Flight *)> Flight::getStringGetter(string attribute)
{
    function<string(Flight *)> func = nullptr;
    if (attribute == "carrier")
    {
        func = &Flight::getCarrier;
    }
    return func;
}