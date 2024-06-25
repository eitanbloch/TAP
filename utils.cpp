#include <iostream>
#include <sstream>
#include <fstream>
#include <chrono>
#include "utils.h"

pair<Graph, Intervals> loadData(const string& filename) {
    Graph graph;
    Intervals intervals;

    // Open the file
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Unable to open file: " << filename << endl;
        exit(1);
    }

    string line;

    // Read the graph
    while (getline(file, line) && line != "-") {
        istringstream iss(line);
        int u, len;
        iss >> u >> len;
        vector<AdjEntry> adjList;
        for (int i = 0; i < len; i++) {
            int v;
            double w;
            iss >> v >> w;
            adjList.emplace_back(v, w);
        }
        graph.push_back(adjList);
    }

    // Read the intervals
    while (getline(file, line) && line != "-") {
        istringstream iss(line);
        int node, n;
        iss >> node >> n;
        IntervalList intervalList;
        for (int i = 0; i < n; i++) {
            double start, end;
            iss >> start >> end;
            intervalList.emplace_back(start, end);
        }
        intervals.push_back(intervalList);
    }
    return {graph, intervals};
}