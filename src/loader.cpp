#include "loader.h"

#include "multigraph.h"
#include "airport.h"
#include "flight.h"
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

string NODES_FILE = "../data/airports_actually_used.csv";
string EDGES_FILE = "../data/flights.csv";

void createNodes(Multigraph &multigraph, vector<vector<string>> data)
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

void createEdges(Multigraph &multigraph, vector<vector<string>> data)
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

void loadData(Multigraph &multigraph, int argc, char *argv[])
{
    string nodes_file = NODES_FILE;
    string edges_file = EDGES_FILE;
    if (argc == 3)
    {
        nodes_file = argv[1];
        edges_file = argv[2];
    }

    auto node_data = loadCSV(nodes_file);
    createNodes(multigraph, node_data);

    auto edge_data = loadCSV(edges_file);
    createEdges(multigraph, edge_data);
}