// Menu for filters and weighters getter
#include "edge.h"
#include "node.h"
#include "flight.h"
#include "airport.h"
#include "multigraph.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <functional>
#include <limits>
#include <vector>

using namespace std;

string NODES_FILE = "../data/airports_actually_used.csv";
string EDGES_FILE = "../data/flights.csv";

Multigraph multigraph = Multigraph();
int chosenFunction = -1;

// Choose filter
// Choose Weighter

int getIntInput(string message, int inputMin, int inputMax)
{
    int input;
    cout << message;
    while (1)
    {
        while (!(cin >> input))
        // while (input < inputMin || input > inputMax)
        {
            cout << "Invalid input. Try again: " << endl;
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
        }

        if (input < inputMin || input > inputMax)
        {
            cout << "Must be between " << inputMin << " and " << inputMax << endl;
            continue;
        }
        break;
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
    int choice = getIntInput("Enter your choice: ", 0, 9);
    if (choice == 0)
    {
        cout << endl
             << endl;
        return nullptr;
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

        while (!(cin >> choice) || choice < 0 || choice > 6)
        {
            cout << "Invalid input. Try again: ";
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
        }

        // auto weighter = Flight::getWeighter("dayMonth"); -> need to do this function
        // Edge::getEdgeWeighter(weighter);

        if (choice == 0)
        {
            cout << endl
                 << endl;
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

// function<vector<vector<Edge *>>> chooseProblem()

void chooseProblem()
{
    cout << "1 - Shortest Path by Dijkstra Algorithm" << endl;
    cout << "2 - Local Minimum Spanning Tree" << endl;
    cout << "0 - Exit" << endl;

    int choice = getIntInput("Choose the problem you wish to solve: ", 0, 2);

    if (choice == 0)
    {
        chosenFunction = 0;
        return;
    }
    else if (choice == 1)
    {
        chosenFunction = 1;
        vector<Node *> nodes = multigraph.getNodes();
        return;

        // TODO: add the dijkstra function
    }
    else if (choice == 2)
    {
        chosenFunction = 2;
        return;
        // TODO: add the local minimum spanning tree function
    }
}

void viewNodes()
{
    int choice;
    int doViewNode = true;
    while (doViewNode)
    {
        cout << "---- VIEW NODES ----" << endl;
        cout << "1 - View node by Id" << endl;
        cout << "0 - Exit" << endl;
        choice = getIntInput("Enter your choice: ", 0, 1);

        if (choice == 1)
        {
            int nodeId = getIntInput("Enter node id: ");
            Node *node = multigraph.getNode(nodeId);
            if (node == nullptr)
            {
                cout << "Node not found" << endl;
                continue;
            }
            node->print();
        }
        else if (choice == 0)
        {
            doViewNode = false;
        }
        else
        {
            cout << "Invalid input. Try again: " << endl;
            break;
        }
    }
}

void viewEdgesFilter(vector<Edge *> edges)
{
    int choice;
    while (true)
    {
        cout << "---- FILTER EDGES ---- " << endl;
        cout << "1 - Filter " << endl;
        cout << "0 - Exit" << endl;

        choice = getIntInput("Enter your choice: ", 0, 1);

        if (choice == 1)
        {
            std::function<bool(Edge *)> filter = chooseFilter();
            if (filter == nullptr)
            {
                break;
            }
            int count = 0;
            for (Edge *edge : edges)
            {
                if (filter(edge))
                {
                    count++;
                    cout << edge->toString() << endl;
                }
            }
            cout << "Number of edges that satisfy parameters: " << count << endl;
        }
        else
        {
            break;
        }
    }
    return;
}

void viewEdges()
{
    int choice;
    bool doViewEdge = true;
    while (doViewEdge)
    {
        cout << "---- VIEW EDGES ----" << endl;
        cout << "1 - View edge by Id" << endl;
        choice = getIntInput("Enter your choice: ", 0, 1);

        if (choice == 1)
        {
            bool doViewEdgeById = true;
            Node *origin = nullptr;
            Node *destination = nullptr;
            vector<Edge *> edges;
            int choice2;
            while (doViewEdgeById)
            {
                cout << "---- VIEW EDGES BY ID ---- " << endl;
                cout << "1 - Choose origin node" << endl;
                cout << "2 - Choose destination node" << endl;
                cout << "3 - View edge" << endl;
                cout << "0 - Return " << endl;
                choice2 = getIntInput("Enter your choice: ", 0, 3);

                if (choice2 == 1)
                {
                    origin = multigraph.getNode(getIntInput("Enter origin node id: "));
                    if (origin == nullptr)
                    {
                        cout << "No node found " << endl;
                        continue;
                    }
                }
                else if (choice2 == 2)
                {
                    destination = multigraph.getNode(getIntInput("Enter destination node id: "));
                    if (destination == nullptr)
                    {
                        cout << "No node found " << endl;
                        continue;
                    }
                }
                else if (choice2 == 3)
                {
                    if (origin == nullptr || destination == nullptr)
                    {
                        cout << "Please select origin and destination nodes" << endl;
                        continue;
                    }
                    cout << "Origin: " << origin->toString() << endl;
                    cout << "Destination: " << destination->toString() << endl;
                    edges = origin->getOutgoingEdgesToNode(destination);
                    cout << "Found " << edges.size() << " edges: " << endl;
                    for (auto edge : edges)
                    {
                        cout << edge->toString() << endl;
                    }
                    viewEdgesFilter(edges);
                }
                else if (choice2 == 0)
                {
                    doViewEdgeById = false;
                }
                else
                {
                    cout << "Please select a valid option" << endl;
                }
            }
        }
        else if (choice == 0)
        {
            doViewEdge = false;
        }
        else
        {
            cout << "Invalid input. Try again: " << endl;
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
        chooseProblem();
        break;
    case 4:
        // chooseOriginDest();
        break;
    case 5:
        // run();
        break;
    case 6: // debugging purposes
        viewNodes();
        break;
    case 7: // debugging purposes
        viewEdges();
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
        cout << "6 - View nodes" << endl;
        cout << "7 - View edges" << endl;
        cout << "0 - Exit" << endl;
        cout << endl;

        choice = getIntInput("Enter your choice: ", 0, 7);
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

vector<vector<string>> loadCSV(string fname)
{
    vector<vector<string>> content;
    vector<string> row;
    string line, word;

    fstream file(fname, ios::in);
    if (file.is_open())
    {
        while (getline(file, line))
        {
            row.clear();

            stringstream str(line);

            while (getline(str, word, ','))
                row.push_back(word);
            content.push_back(row);
        }
    }
    else
    {
        cout << "Could not open the file\n";
    }
    file.close();

    return content;
}

void createNodes(vector<vector<string>> data)
{
    for (int i = 1; i < data.size(); i++)
    {
        double lon, lat;
        int id;
        string code, name, city, state, stateName;

        id = stoi(data[i][0]);
        code = data[i][1];
        name = data[i][2];
        city = data[i][3];
        state = data[i][4];
        stateName = data[i][5];
        lat = stod(data[i][6]);
        lon = stod(data[i][7]);

        Airport airport = Airport(lon, lat, id, code, name, city, state, stateName);
        multigraph.createNode(airport);
    }
}

void createEdges(vector<vector<string>> data)
{
    for (int i = 1; i < data.size(); i++)
    {
        int dayMonth, dayWeek, originId, destId, depDelay, arrDelay;
        double distance, flightTime;
        string carrier;

        dayMonth = stoi(data[i][0]);
        dayWeek = stoi(data[i][1]);
        carrier = data[i][2];
        originId = stoi(data[i][3]);
        destId = stoi(data[i][4]);
        depDelay = stoi(data[i][5]);
        arrDelay = stoi(data[i][6]);
        distance = stod(data[i][7]);
        flightTime = stod(data[i][8]);
        Flight flight = Flight(dayMonth, dayWeek, carrier, originId, destId, depDelay, arrDelay, distance, flightTime);
        Node *originNode = multigraph.getNode(originId);
        Node *destinationNode = multigraph.getNode(destId);
        multigraph.createEdge(originNode, destinationNode, flight);
    }
}

void loadData(int argc, char *argv[])
{
    string nodes_file = NODES_FILE;
    string edges_file = EDGES_FILE;
    if (argc == 3)
    {
        nodes_file = argv[1];
        edges_file = argv[2];
    }

    auto node_data = loadCSV(nodes_file);
    createNodes(node_data);

    auto edge_data = loadCSV(edges_file);
    createEdges(edge_data);
}

int main(int argc, char *argv[])
{
    cout << "Loading data... " << endl;
    loadData(argc, argv);
    cout << "Data loaded. Creating multigraph" << endl;
    startMultigraph();
    menu();
    return 0;
}