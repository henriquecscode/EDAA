// Menu for filters and weighters getter
#include "edge.h"
#include "node.h"
#include "flight.h"
#include "multigraph.h"
#include "attribute_type.h"
#include "loader.h"
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <functional>
#include <limits>
#include <vector>
#include <stdio.h>
#include <cstring>

using namespace std;

streambuf *terminalstream = cin.rdbuf();
streambuf *inputstream;

Multigraph multigraph = Multigraph();
int problem = -1;
EdgeFilter filter;
EdgeWeighter weighter;

string pythonScriptPath = "python3 ../gui/gui.py";

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
            if (cin.rdbuf() == inputstream)
            {
                cin.rdbuf(terminalstream);
                continue;
            }
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

EdgeFilter chooseFilter()
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
    cout << "10 - No filter" << endl;
    cout << "0 - Exit" << endl;
    cout << endl;

    // do a safe input here
    int choice = getIntInput("Enter your choice: ", 0, 10);
    cout << endl
         << endl;

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
    else if (choice == 10)
    {
        return Edge::getEdgeFilter();
    }
    else
    {
        cout << "Invalid input. Try again: ";
        return nullptr;
    }
}

EdgeWeighter chooseWeighter()
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

int chooseProblem()
{
    cout << "1 - Shortest Path by Dijkstra Algorithm" << endl;
    cout << "2 - Local Minimum Spanning Tree" << endl;
    cout << "3 - Erdos" << endl;
    cout << "0 - Exit" << endl;

    int choice = getIntInput("Choose the problem you wish to solve: ", 0, 3);

    if (choice == 0)
    {
        return -1;
    }
    else
    {
        return choice;
    }
}

void viewSolution(pair<vector<Edge *>, int> solution)
{
    int steps = solution.second;
    vector<Edge *> edges = solution.first;
    // iterate edges, please
    for (Edge *edge : edges)
    {
        cout << edge->getSource()->getData().getId() << " -> " << edge->getDest()->getData().getId() << endl;
    }
    cout << "Found above solution (top to bottom)" << endl;
    cout << "Total steps: " << steps << endl;
    cout << endl
         << endl;
}

void viewGUISolution(pair<vector<Edge *>, int> solution)
{
    string solutionName = "solution_file.txt";
    string path = "../data/solutions/";
    ofstream solutionFile(path + solutionName);

    vector<Edge *> edges = solution.first;
    solutionFile << edges.size() << '\n';
    for (Edge *edge : edges)
    {
        solutionFile << edge->toCSV() << '\n';
    }
    solutionFile.flush();

    system((pythonScriptPath + " " + solutionName).c_str());
    // execl(pythonScriptPath.c_str(), solutionName.c_str());
}

void run()
{
    if (problem == 1)
    {
        // djsktra
        if (filter == nullptr)
        {
            cout << "You must choose a filter first" << endl;
            return;
        }
        if (weighter == nullptr)
        {
            cout << "You must choose a weighter first" << endl;
            return;
        }
    }
    else if (problem == 2)
    {
        // spanning tree
        if (filter == nullptr)
        {
            cout << "You must choose a filter first" << endl;
            return;
        }
        if (weighter == nullptr)
        {
            cout << "You must choose a weighter first" << endl;
            return;
        }
    }
    else if (problem == 3)
    {
        // erdos
        if (filter == nullptr)
        {
            cout << "You must choose a filter first" << endl;
            return;
        }
    }
    else
    {
        cout << "You must choose a problem first" << endl;
        return;
    }

    int choice;
    Node *origin = nullptr;
    Node *destination = nullptr;
    int algorithm = 0, edgeCollector = 0;
    while (true)
    {
        cout << "---- Choose parameters ---- " << endl;
        cout << "1 - Choose node" << endl;
        cout << "2 - Choose destination node" << endl;
        cout << "3 - Choose algorithm" << endl;
        cout << "4 - Run" << endl;
        if (problem == 2)
        {
            cout << "5 - Choose edge collector" << endl;
        }
        cout << "0 - Return " << endl;
        choice = getIntInput("Enter your choice: ", 0, problem == 2 ? 5 : 4);
        cout << endl
             << endl;

        if (choice == 1)
        {
            origin = multigraph.getNode(getIntInput("Enter origin node id: "));
            if (origin == nullptr)
            {
                cout << "No node found " << endl;
                cout << endl
                     << endl;
                continue;
            }
        }
        else if (choice == 2)
        {
            destination = multigraph.getNode(getIntInput("Enter destination node id: "));
            if (destination == nullptr)
            {
                cout << "No node found " << endl;
                cout << endl
                     << endl;
                continue;
            }
        }
        else if (choice == 3)
        {
            int max = 0;
            cout << "---- Choose algorithm ----" << endl;
            if (problem == 1)
            {
                cout << "1 - dijkstra" << endl;
                cout << "2 - dijkstra by node" << endl;
                max = 2;
            }
            else if (problem == 2)
            {
                cout << "1 - bfs" << endl;
                cout << "2 - bfs by node" << endl;
                max = 2;
            }
            else if (problem == 3)
            {
                cout << "1 - bfs " << endl;
                cout << "2 - bfs by node" << endl;
                max = 2;
            }

            cout << "0 - Return " << endl;

            algorithm = getIntInput("Enter your choice: ", 0, max);
            cout << endl
                 << endl;
            continue;
        }
        else if (choice == 4)
        {
            if (origin == nullptr)
            {
                cout << "You must choose an origin node" << endl;
                cout << endl
                     << endl;
                continue;
            }
            if (destination == nullptr)
            {
                if (problem != 2)
                {
                    cout << "You must choose a destination node" << endl;
                    cout << endl
                         << endl;
                    continue;
                }
            }
            if (algorithm == 0)
            {
                cout << "You must choose an algorithm" << endl;
                cout << endl
                     << endl;
                continue;
            }
            break;
        }
        else if (choice == 5)
        {
            cout << "---- Choose edge collector ----" << endl;
            if (problem == 2)
            {
                cout << "1 - no collector" << endl;
                cout << "2 - best edge" << endl;
            }
            edgeCollector = getIntInput("Enter your choice: ", 1, 2);
        }
        else if (choice == 0)
        {
            return;
        }
        else
        {
            cout << "Please select a valid option" << endl;
        }
    }

    pair<vector<Edge *>, int> solution;

    if (problem == 1)
    {
        // dijkstra
        vector<Node *> nodes = {origin, destination};
        vector<vector<Edge *>> allPaths = multigraph.getShortestPathDijkstra(nodes, filter, weighter, algorithm);
        solution = make_pair(allPaths[0], allPaths[0].size());
    }
    else if (problem == 2)
    {
        algorithm = algorithm << 0;
        edgeCollector = edgeCollector << 2;
        int algorithmCodification = algorithm | edgeCollector;
        // spanning tree
        vector<Edge *> path = multigraph.getLocalMinimumSpanningTree(origin, filter, weighter, algorithmCodification);
        solution = make_pair(path, path.size());
    }
    else if (problem == 3)
    {
        solution = multigraph.getErdos(origin, destination, filter, algorithm);
    }
    viewSolution(solution);
    viewGUISolution(solution);
}

void viewNodesFilter(Node *node)
{
    cout << " ---- VIEW THIS NODE ---- " << endl;
    cout << "1 - View all edges" << endl;
    cout << "2 - View incoming edges" << endl;
    cout << "3 - View outgoing edges" << endl;
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
                cout << "Node not found." << endl;
                cout << endl
                     << endl;
                continue;
            }
            viewNodesFilter(node);
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
            EdgeFilter filter = chooseFilter();
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
                        cout << "No node found." << endl;
                        cout << endl
                             << endl;
                        continue;
                    }
                }
                else if (choice2 == 2)
                {
                    destination = multigraph.getNode(getIntInput("Enter destination node id: "));
                    if (destination == nullptr)
                    {
                        cout << "No node found." << endl;
                        cout << endl
                             << endl;
                        continue;
                    }
                }
                else if (choice2 == 3)
                {
                    if (origin == nullptr || destination == nullptr)
                    {
                        cout << "Please select origin and destination nodes..." << endl;
                        cout << endl
                             << endl;
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
                    cout << "Please select a valid option!" << endl;
                    cout << endl
                         << endl;
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
        filter = chooseFilter();
        break;
    case 2:
        weighter = chooseWeighter();
        break;
    case 3:
        problem = chooseProblem();
        break;
    case 4:
        // chooseOriginDest();
        break;
    case 5:
        run();
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
        cout << endl
             << endl;
        doChoice(choice);
    }
}

int main(int argc, char *argv[])
{
    istringstream iss;
    if (argc > 1)
    {
        // iss =istringstream("1 10 2 5 3 1 5 1 11433 2 13930 3 1 4");
        // iss = istringstream("1 10 2 5 3 1 5 1 11433 2 10140 3 1 4");
        // iss = istringstream("1 10 3 3 5 1 11433 2 10140 3 1 4");
        // iss = istringstream("1 10 3 3 5 1 11433 2 10140 3 2 4");
        iss = istringstream("1 10 2 5 3 2 5 1 11433 3 1 5 2 4");
        inputstream = iss.rdbuf();
        cin.rdbuf(iss.rdbuf());
    }
    cout << "Loading data... " << endl;
    loadData(multigraph, argc, argv);
    cout << "Data loaded. Creating multigraph" << endl;
    menu();
    return 0;
}