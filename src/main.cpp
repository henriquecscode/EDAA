// Menu for filters and weighters getter
#include "edge.h"
#include "flight.h"
#include "airport.h"
#include "multigraph.h"
#include <string>
#include <iostream>
#include <functional>
#include <limits>
using namespace std;

Multigraph multigraph = Multigraph();

// Choose filter
// Choose Weighter

int getIntInput(string message, int inputMin, int inputMax)
{
    int input;
    cout << message;
    while (!(cin >> input) || input <= inputMin || input >= inputMax)
    {
        cout << "Invalid input. Try again: ";
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }
    return input;
}
int getIntInput(string message)
{
    return getIntInput(message, numeric_limits<int>::min(), numeric_limits<int>::max());
}
int getIntInput(string message, int inpuMin)
{
    return getIntInput(message, inpuMin, numeric_limits<int>::max());
}

double getDoubleInput(string message, double inputMin, double inputMax)
{
    double input;
    cout << message;
    while (!(cin >> input) || input <= inputMin || input >= inputMax)
    {
        cout << "Invalid input. Try again: ";
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
    }
    return input;
}
double getDoubleInput(string message)
{
    return getDoubleInput(message, numeric_limits<double>::min(), numeric_limits<double>::max());
}
double getDoubleInput(string message, double inputMin)
{
    return getDoubleInput(message, inputMin, numeric_limits<double>::max());
}

function<bool(Edge *)> chooseFilter()
{
    string attributes[] = {"dayMonth", "dayWeek", "carrier", "originId", "destId", "depDelay", "arrDelay", "distance", "flightTime"};
    while (1)
    {
        cout << "Choose attribute to filter by:" << endl;
        cout << "1 - Filter by dayMonth" << endl;
        cout << "2 - Filter by dayWeek" << endl;
        cout << "3 - Filter by carrier" << endl;
        cout << "4 - Filter by originId" << endl;
        cout << "5 - Filter by destId" << endl;
        cout << "6 - Filter by depDelay" << endl;
        cout << "7 - Filter by arrDelay" << endl;
        cout << "8 - Filter by distance" << endl;
        cout << "9 - Filter by flightTime" << endl;
        cout << "0 - Exit" << endl;
        cout << endl;

        // do a safe input here
        int choice;
        getIntInput("Enter your choice: ", 0, 9);
        if (choice == 0)
        {
            return Edge::getEdgeFilter();
        }
        else if (choice == 1 || choice == 2 || choice == 4 || choice == 5 || choice == 6 || choice == 7)
        {
            int min, max;
            min = getIntInput("Enter minimum" + attributes[choice - 1] + ": ");
            max = getIntInput("Enter maximum" + attributes[choice - 1] + ": ");
            auto dataGetter = Flight::getIntGetter(attributes[choice - 1]);
            return Edge::getEdgeFilter(dataGetter, min, max);
        }
        else if (choice == 8 || choice == 9)
        {
            double min, max;
            min = getDoubleInput("Enter minimum" + attributes[choice - 1] + ": ");
            max = getDoubleInput("Enter maximum" + attributes[choice - 1] + ": ");
            auto dataGetter = Flight::getDoubleGetter(attributes[choice - 1]);
            return Edge::getEdgeFilter(dataGetter, min, max);
        }
        else if (choice == 3)
        {
            string carrier;
            cout << "Enter carrier: ";
            cin >> carrier;
            auto dataGetter = Flight::getStringGetter("carrier");
            return Edge::getEdgeFilter(dataGetter, carrier);
        }
        else
        {
            cout << "Invalid input. Try again: ";
        }
    }
}

function<bool(Edge *)> chooseWeighter()
{
    while (1)
    {
        cout << "Choose the attribute to give a weight to:" << endl;
        cout << "1 - dayMonth" << endl;
        cout << "2 - dayWeek" << endl;
        cout << "3 - depDelay" << endl;
        cout << "4 - arrDelay" << endl;
        cout << "5 - distance" << endl;
        cout << "6 - flightTime" << endl;
        cout << "0 - Exit" << endl;
        cout << endl;

        int choice;
        cout << "Enter your choice: ";

        while (!(cin >> choice) || choice < 0 || choice > 9)
        {
            cout << "Invalid input. Try again: ";
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
        }

        // auto weighter = Flight::getWeighter("dayMonth"); -> need to do this function
        // Edge::getEdgeWeighter(weighter);

        if (choice == 0)
        {
            return nullptr;
        }
        else if (choice == 1)
        {
            return Edge::getEdgeWeighter("dayMonth");
        }
        else if (choice == 2)
        {
            return Edge::getEdgeWeighter("dayWeek");
        }
        else if (choice == 3)
        {
            return Edge::getEdgeWeighter("depDelay");
        }
        else if (choice == 4)
        {
            return Edge::getEdgeWeighter("arrDelay");
        }
        else if (choice == 5)
        {
            return Edge::getEdgeWeighter("distance");
        }
        else if (choice == 6)
        {
            return Edge::getEdgeWeighter("flightTime");
        }
        else
        {
            cout << "Invalid input. Try again: ";
        }
    }
}

void doChoice(int choice)
{
    switch (choice)
    {
    case 1:
        chooseFilter();
        break;
    case 2:
        chooseWeighter();
        break;
    case 3:
        // chooseProblem();
        break;
    case 4:
        // chooseOriginDest();
        break;
    case 5:
        // run();
        break;
    case 0:
        exit(0);
        break;
    default:
        cout << "Invalid input. Try again: ";
        break;
    }
}

void menu()
{
    auto filter = Edge::getEdgeFilter();
    auto weighter = Edge::getEdgeWeighter("distance");
    int choice;
    multigraph.getBestEdges(&filter, &weighter);

    while (1)
    {
        cout << "-----  AIRPORT FLIGHTS MULTIGRAPH  -----" << endl;
        cout << "1 - Choose filter" << endl;
        cout << "2 - Choose weighter" << endl;
        cout << "3 - Choose problem to solve" << endl;
        cout << "4 - Choose origin and destination" << endl;
        cout << "5 - Run!" << endl;
        cout << "0 - Exit" << endl;
        cout << endl;

        choice = getIntInput("Enter your choice", 0, 2);
        doChoice(choice);
    }
}

void startMultigraph()
{
    // static init
    Flight flight1 = Flight(1, 1, "CARRIER", 1, 2, 1, 2, 1, 2),
           flight2 = Flight(2, 2, "CARRIER2", 1, 2, -2, -3, -1, 2);

    Airport airport1 = Airport(1, 2),
            airport2 = Airport(2, 1);

    auto node = multigraph.createNode(airport1);
    auto node2 = multigraph.createNode(airport2);

    multigraph.createEdge(node, node2, flight1);
    multigraph.createEdge(node, node2, flight2);
}

int main()
{
    cout << "-----  AIRPORT FLIGHTS MULTIGRAPH  -----" << endl;
    startMultigraph();
    menu();
    return 0;
}