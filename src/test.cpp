#include <random>
#include <fstream>
#include <string>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
// #include <direct.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include "loader.h"
#include "edge.h"
#include "node.h"
#include "multigraph.h"
#include <functional>

using namespace std;
unsigned int SEED = 42;
unsigned int DEFAULT_NUMBER_OF_PAIRS = 3;

string run_prefix = "";
string DATA_DIR = "data/";
string DIJKSTRA_DIR = DATA_DIR + "dijkstra/";
string ERDOS_DIR = DATA_DIR + "erdos/";
string BFS_DIR = DATA_DIR + "bfs/";
string SPANNING_TREE_DIR = DATA_DIR + "spanning_tree/";

Multigraph multigraph;
typedef vector<vector<Edge *>> (Multigraph::*MultiNodeProblem)(vector<Node *>, EdgeFilter, EdgeWeighter, int);

string getDate()
{
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S");
    string str = oss.str();
    return str;
}

void initDirs()
{
    int check;
    check = mkdir(DATA_DIR.c_str(), 0777);

    check = mkdir(DIJKSTRA_DIR.c_str(), 0777);
    check = mkdir(ERDOS_DIR.c_str(), 0777);
    check = mkdir(BFS_DIR.c_str(), 0777);
    check = mkdir(SPANNING_TREE_DIR.c_str(), 0777);
}

Node *getRandomNode()
{
    // call function to get random node
    auto nodes = multigraph.getNodes();
    int randomIndex = rand() % nodes.size();
    return nodes[randomIndex];
}

vector<pair<Node *, Node *>> getPairNodesForTesting(int numberOfPairs)
{
    vector<pair<Node *, Node *>> pairs;
    for (int i = 0; i < numberOfPairs; i++)
    {
        Node *origin;
        Node *destination;
        while (true)
        {
            origin = getRandomNode();
            destination = getRandomNode();
            if(origin != destination){
                break;
            }
        }
        pairs.push_back(make_pair(origin, destination));
    }
    return pairs;
}
vector<pair<Node *, Node *>> getPairNodesForTesting()
{
    return getPairNodesForTesting(DEFAULT_NUMBER_OF_PAIRS);
}

vector<pair<string, EdgeFilter>> getEdgeFilters()
{
    int MIN_DEP_DELAY = -100;
    int MAX_DEP_DELAY = 2000;
    int min, max;
    vector<pair<string, EdgeFilter>> edgeFilters;
    EdgeFilter filter;
    string filterName;
    filter = Edge::getEdgeFilter();
    filterName = "noFilter";
    edgeFilters.push_back(make_pair(filterName, filter));
    auto dataGetter = Flight::getIntGetter("depDelay");

    min = MIN_DEP_DELAY;
    max = (int)(0.7 * (MAX_DEP_DELAY - MIN_DEP_DELAY) + MIN_DEP_DELAY);
    filter = Edge::getEdgeFilter(dataGetter, min, max);
    filterName = "depDelayFilter1";
    edgeFilters.push_back(make_pair(filterName, filter));

    max = (int)(0.5 * (MAX_DEP_DELAY - MIN_DEP_DELAY) + MIN_DEP_DELAY);
    filter = Edge::getEdgeFilter(dataGetter, min, max);
    filterName = "depDelayFilter2";
    edgeFilters.push_back(make_pair(filterName, filter));

    max = (int)(0.3 * (MAX_DEP_DELAY - MIN_DEP_DELAY) + MIN_DEP_DELAY);
    filter = Edge::getEdgeFilter(dataGetter, min, max);
    filterName = "depDelayFilter3";
    edgeFilters.push_back(make_pair(filterName, filter));

    max = (int)(0.1 * (MAX_DEP_DELAY - MIN_DEP_DELAY) + MIN_DEP_DELAY);
    filter = Edge::getEdgeFilter(dataGetter, min, max);
    filterName = "depDelayFilter4";
    edgeFilters.push_back(make_pair(filterName, filter));

    max = (int)(0.01 * (MAX_DEP_DELAY - MIN_DEP_DELAY) + MIN_DEP_DELAY);
    filter = Edge::getEdgeFilter(dataGetter, min, max);
    filterName = "depDelayFilter5";
    edgeFilters.push_back(make_pair(filterName, filter));

    return edgeFilters;
}

vector<pair<string, EdgeWeighter>> getEdgeWeighters()
{
    vector<pair<string, EdgeWeighter>> edgeWeighters;
    EdgeWeighter weighter;
    string weighterName;
    weighter = Edge::getEdgeWeighter("distance");
    weighterName = "distanceWeighter";
    edgeWeighters.push_back(make_pair(weighterName, weighter));

    return edgeWeighters;
}

vector<pair<string, EdgeWeighter>> getNoWeighters()
{
    return {make_pair("noWeighter", nullptr)};
}
// typedef void (*MultiNodeProblem)(vector<Node *>, EdgeFilter, EdgeWeighter, int);

void loop(vector<pair<int, string>> algorithmPairs,
          vector<pair<string, EdgeFilter>> edgeFilters,
          vector<pair<string, EdgeWeighter>> edgeWeighters,
          vector<pair<Node *, Node *>> pairs,
          MultiNodeProblem problem,
          string problem_dir)
{
    string date = getDate();
    for (auto algorithmPair : algorithmPairs)
    {
        int algorithmNumber = algorithmPair.first;
        string algorithmName = algorithmPair.second;

        for (auto pairNameFilter : edgeFilters)
        {
            string filterName = pairNameFilter.first;
            EdgeFilter filter = pairNameFilter.second;
            for (auto pairNameWeighter : edgeWeighters)
            {
                string weighterName = pairNameWeighter.first;
                EdgeWeighter weighter = pairNameWeighter.second;

                string file_run_prefix = run_prefix != "" ? run_prefix + "_" : "";
                string filedir = problem_dir + file_run_prefix + date + "_" + filterName + "_" + weighterName + "_" + algorithmName + "/";
                string logFilename = filedir + "log.csv";
                string totalFilename = filedir + "total.csv";
                mkdir(filedir.c_str(), 0777);

                ofstream file(logFilename);
                ofstream totalFile(totalFilename);
                file << "origin, destination, duration(mu_s)\n";
                long long total = 0;
                long long totalAverage;
                for (auto pair : pairs)
                {
                    Node *origin = pair.first;
                    Node *destination = pair.second;
                    vector<Node *> nodes = {origin, destination};
                    auto start = std::chrono::high_resolution_clock::now();
                    (multigraph.*problem)(nodes, filter, weighter, algorithmNumber);
                    auto finish = std::chrono::high_resolution_clock::now();
                    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
                    total += microseconds;
                    file << origin->getData().getId() << "," << destination->getData().getId() << "," << microseconds << endl;
                }
                totalAverage = total / pairs.size();
                totalFile << "total(mu_s), numberOfRuns, totalAverage(mu_s)\n";
                totalFile << total << "," << pairs.size() << "," << totalAverage << endl;

                file.close();
                totalFile.close();
            }
        }
    }
}
/*
void loop(vector<pair<int, string>> algorithmPairs,
          vector<pair<string, EdgeFilter>> edgeFilters,
          vector<pair<Node *, Node *>> pairs)
{
    for (auto algorithmPair : algorithmPairs)
    {
        int algorithmNumber = algorithmPair.first;
        string algorithmName = algorithmPair.second;

        for (auto pairNameFilter : edgeFilters)
        {
            string filterName = pairNameFilter.first;
            EdgeFilter filter = pairNameFilter.second;

            string weighterName = pairNameWeigter.first;
            EdgeWeighter weighter = pairNameWeigter.second;

            string date = getDate();
            string filedir = DIJKSTRA_DIR + date + "_" + filterName + "_" + algorithmName + "/";
            string logFilename = filedir + "log.csv";
            string totalFilename = filedir + "total.csv";
            mkdir(filedir.c_str(), 0777);

            ofstream file(logFilename);
            ofstream totalFile(totalFilename);
            file << "origin, destination, duration(mu_s)\n";
            long long total = 0;
            long long totalAverage;
            for (auto pair : pairs)
            {
                Node *origin = pair.first;
                Node *destination = pair.second;
                auto start = std::chrono::high_resolution_clock::now();
                (multigraph.*problem)(origin, destination, filter);
                auto finish = std::chrono::high_resolution_clock::now();
                auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count();
                total += microseconds;
                file << origin->getData().getId() << "," << destination->getData().getId() << "," << microseconds << endl;
            }
            totalAverage = total / pairs.size();
            totalFile << "total(mu_s), numberOfRuns, totalAverage(mu_s)\n";
            totalFile << total << "," << pairs.size() << "," << totalAverage << endl;

            file.close();
            totalFile.close();
        }
    }
}
*/
void testDijkstra()
{
    vector<pair<int, string>> algorithmPairs = {
        make_pair(1, "bfs"),
        make_pair(2, "bfsByNode")};
    vector<pair<string, EdgeFilter>> edgeFilters = getEdgeFilters();
    vector<pair<string, EdgeWeighter>> edgeWeighters = getEdgeWeighters();
    vector<pair<Node *, Node *>> pairs = getPairNodesForTesting();

    std::cout << "Starting dijskstra test" << endl;
    // std::function<vector<vector<Edge*>>(vector<Node*>, EdgeFilter, EdgeWeighter, int)>problem = Multigraph::getShortestPathDijkstra;
    // vector<vector<Edge *>> (Multigraph::*problem)(vector<Node *>, EdgeFilter, EdgeWeighter, int) = &Multigraph::getShortestPathDijkstra;
    // MultiNodeProblem problem[](vector<Node *> nodes, EdgeFilter filter, EdgeWeighter weighter, int algorithmNumber)
    // {
    //     multigraph.getShortestPathDijkstra(nodes, filter, weighter, algorithmNumber);
    // };
    // vector<vector<Edge *>> (Multigraph::*problem)(vector<Node *>, EdgeFilter, EdgeWeighter, int) = &Multigraph::getShortestPathDijkstra;
    MultiNodeProblem problem = &Multigraph::getShortestPathDijkstra;
    loop(algorithmPairs, edgeFilters, edgeWeighters, pairs, problem, DIJKSTRA_DIR);
    std::cout << "Finished dijkstra test" << endl;
}

void testErdos()
{
    vector<pair<int, string>> algorithmNPairs = {
        make_pair(1, "erdos"),
        make_pair(2, "erdosByNode")};
    vector<pair<string, EdgeFilter>> edgeFilters = getEdgeFilters();
    vector<pair<string, EdgeWeighter>> edgeWeighters = getEdgeWeighters();
    vector<pair<Node *, Node *>> pairs = getPairNodesForTesting();
    std::cout << "Starting erdos test" << endl;
    MultiNodeProblem problem = &Multigraph::getErdos;
    loop(algorithmNPairs, edgeFilters, edgeWeighters, pairs, problem, ERDOS_DIR);
    std::cout << "Finished erdos test" << endl;
}
void testBfs()
{
    vector<pair<int, string>> algorithmNPairs = {
        make_pair(1, "bfs"),
        make_pair(2, "bfsByNode")};
    vector<pair<string, EdgeFilter>> edgeFilters = getEdgeFilters();
    vector<pair<string, EdgeWeighter>> edgeWeighters = getEdgeWeighters();
    vector<pair<Node *, Node *>> pairs = getPairNodesForTesting();
    std::cout << "Starting bfs test" << endl;
    MultiNodeProblem problem = &Multigraph::getBfs;
    loop(algorithmNPairs, edgeFilters, edgeWeighters, pairs, problem, BFS_DIR);
    std::cout << "Finished bfs test" << endl;
}
void testSpanningTree()
{
    vector<pair<int, string>> algorithmNPairs = {
        // make_pair((1 << 0 | 1 << 2), "Bfs&getEdges"),
        // make_pair((2 << 0 | 1 << 2), "BfsByNode&getEdges"),
        make_pair((1 << 0 | 2 << 2), "Bfs&getBestEdges"),
        // make_pair((2 << 0 | 2 << 2), "BfsByNode&getBestEdges")
    };
    vector<pair<string, EdgeFilter>> edgeFilters = getEdgeFilters();
    vector<pair<string, EdgeWeighter>> edgeWeighters = getEdgeWeighters();
    vector<pair<Node *, Node *>> pairs = getPairNodesForTesting();
    std::cout << "Starting spanning tree test" << endl;
    MultiNodeProblem problem = &Multigraph::getLocalMinimumSpanningTree;
    loop(algorithmNPairs, edgeFilters, edgeWeighters, pairs, problem, SPANNING_TREE_DIR);
    std::cout << "Finished spanning tree test" << endl;
}

int main(int argc, char *argv[])
{
    srand(SEED);
    initDirs();

    std::cout << "Loading data" << endl;
    loadData(multigraph, argc, argv);

    if (argc > 3)
    {
        run_prefix = argv[3];
    }

    std::cout << "Loaded data" << endl;
    testDijkstra();
    testErdos();
    testBfs();
    testSpanningTree();

    return 0;
}
