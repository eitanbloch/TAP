#ifndef TASKASSISTANCE_OPTP_H
#define TASKASSISTANCE_OPTP_H

#include <chrono>
#include "otp.h"
#include "bound.h"

/**
 * Abstract base class for the OPTP (Optimal Path and Timing Profile) solvers
 */
class OPTP {
protected:
    Graph& graph; // The graph to solve the OPTP for.
    Intervals& intervals; // The intervals for each vertex in the graph.

    double max_value; // The maximal value found by the OPTP solver.
    Path max_path; // The path resulting in the maximal value found by the OPTP solver.

    int path_count; // The number of paths evaluated by the OPTP solver.

    std::chrono::high_resolution_clock::time_point start_time; // The start time for the OPTP solver.
    const double time_limit = 3600; // The time limit for the OPTP solver.


public:
    /**
     * Constructs a new OPTP object.
     *
     * @param graph The graph to solve the OPTP for.
     * @param intervals The intervals for each vertex in the graph.
     */
    OPTP(Graph& graph, Intervals& intervals) : graph(graph), intervals(intervals), max_value(0), path_count(0) {};

    /**
     * Runs the OPTP solver.
     *
     * @return The maximum reward obtained by the OPTP solver.
     */
    virtual double run() = 0;

    /**
     * Prints the best path found by the OPTP solver.
     */
    void printBestPath() {
        if (max_path.empty()) {
            cout << "No path found" << endl;
            cout << "path: " << path_count << endl;
            return;
        }
        cout << max_path.front().u;
        for (auto& edge: max_path) {
            cout << " -> " << edge.v;
        }
        cout << endl;
        cout << "path: " << path_count << endl;
    }

    /**
     * Returns the number of paths evaluated by the OPTP solver.
     *
     * @return The number of paths evaluated by the OPTP solver.
     */
    int getPathCount() {
        return path_count;
    }

    /**
       * Returns the best path found by the OPTP solver.
       *
       * @return The best path found by the OPTP solver.
       */
    Path getBestPath() {
        return max_path;
    }
};
#endif //TASKASSISTANCE_OPTP_H
