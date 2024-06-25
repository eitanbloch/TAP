#ifndef TASKASSISTANCE_BRANCH_AND_BOUND_H
#define TASKASSISTANCE_BRANCH_AND_BOUND_H

#include "optp.h"

/**
 * Represents a history of values for each vertex in a graph.
 */
class ValueHistory {
    vector<vector<double>> value_history; // The history of values for each vertex.

public:
    /**
     * Constructs a new ValueHistory object.
     *
     * @param graph_size The number of vertices in the graph.
     */
    explicit ValueHistory(size_t graph_size) {
        for (int i = 0; i < graph_size; i++) {
            value_history.push_back(vector<double>({-2 * EPS}));
        }
    }

    /**
    * Returns the last value for a given vertex.
    *
    * @param u The vertex to get the last value for.
    * @return The last value for the given vertex.
    */
    double last(int u) {
        return value_history[u].back();
    }

    /**
     * Adds a value for a given vertex.
     *
     * @param u The vertex to add the value for.
     * @param value The value to add.
     */
    void add(int u, double value) {
        value_history[u].push_back(value);
    }

    /**
     * Removes the last value for a given vertex.
     *
     * @param u The vertex to removeLast the last value for.
     */
    void removeLast(int u) {
        value_history[u].pop_back();
    }

};

/**
 * A Branch and Bound solver for the OPTP problem
 */
class BranchAndBound : public OPTP {

    double eps; // The epsilon value for the OPTP solver.
    double max_interval_size; // The maximum interval size for the OPTP solver.

    GraphBoundWrapper bound; // The upper bound for the OPTP solver.
    ValueHistory value_history; // The value history for the OPTP solver.

    int dummy_vertex; // A dummy vertex.

    /**
     * Preprocesses the intervals.
     */
    void preprocess_intervals();

    /**
     * Preprocesses the graph.
     */
    void preprocess_graph();

    /**
     * Runs the branch and bound algorithm.
     *
     * @param path The path to start the branch and bound algorithm from.
     * @param path_length the length of the path.
     */
    void branchAndBound(Path& path, double path_length);

    /**
     * Runs the OTP algorithm on a path.
     *
     * @param path The path to run the OTP algorithm on.
     * @param w_min The weight of the last edge to add to the path.
     * @return A pair containing the reward of an optimal timing profile, and the PairList needed to compute the upper bound of a path.
     */
    pair<double, PairList *> runOTP(Path& path, double w_min = 0);

public:
    /**
     * Constructs a new BranchAndBound object.
     *
     * @param graph The graph to solve the OPTP for.
     * @param intervals The intervals for each vertex in the graph.
     * @param previous_max The maximum value obtained on the previous graph.
     * @param eps The epsilon approximation value.
     * @param max_interval_size The maximum interval size.
     */
    BranchAndBound(Graph& graph, Intervals& intervals, double previous_max = 0, double eps = 0,
                   double max_interval_size = 1);

    /**
     * Runs the Branch and Bound OPTP solver.
     *
     * @return The maximum reward obtained by the Branch and Bound solver.
     */
    double run() override;

};

#endif //TASKASSISTANCE_BRANCH_AND_BOUND_H
