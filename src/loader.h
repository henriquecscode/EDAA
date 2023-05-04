#ifndef LOADER_H
#define LOADER_H

#include <vector>
#include <string>
#include "multigraph.h"

void createNodes(Multigraph &multigraph, vector<vector<string>> data);

void createEdges(Multigraph &multigraph, vector<vector<string>> data);

vector<vector<string>> loadCSV(string fname);

void loadData(Multigraph &multigraph, int argc, char *argv[]);

#endif