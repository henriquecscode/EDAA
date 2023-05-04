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
#include <stdio.h>
#include <stdlib.h>

using namespace std;
unsigned int SEED = 42;

string DATA_DIR = "data/";
string DIJKSTRA_DIR = DATA_DIR + "dijkstra/";
string DFS_DIR = DATA_DIR + "dfs/";
string ERDOS_DIR = DATA_DIR + "erdos/";
string BFS_DIR = DATA_DIR + "bfs/";
string SPANNING_TREE_DIR = DATA_DIR + "spanning_tree/";

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
    check = mkdir(DFS_DIR.c_str(), 0777);
    check = mkdir(ERDOS_DIR.c_str(), 0777);
    check = mkdir(BFS_DIR.c_str(), 0777);
    check = mkdir(SPANNING_TREE_DIR.c_str(), 0777);
}

void testDijkstra()
{
}
void testDfs()
{
}
void testErdos()
{
}
void testBfs()
{
}
void testSpanningTree()
{
}

int main()
{
    srand(SEED);
    initDirs();

    testDijkstra();
    testDfs();
    testErdos();
    testBfs();
    testSpanningTree();

    return 0;
}
