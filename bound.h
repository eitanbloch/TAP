#ifndef TASKASSISTANCE_BOUND_H
#define TASKASSISTANCE_BOUND_H

#include <iostream>
#include <map>
#include <algorithm>
#include "utils.h"

#define INF 1e9 // Represents infinity.

typedef vector<vector<double>> Matrix; // Represents a 2D matrix of doubles.

/**
 * Function to find the shortest path from a starting node to all other nodes in the graph using Dijkstra's algorithm.
 * @param graph The graph to perform the algorithm on.
 * @param start The starting node.
 * @param dist The vector to store the shortest distances.
 */
void dijkstra(Graph& graph, int start, vector<double>& dist);

/**
 * Represents an interval associated with a vertex.
 */
class VInterval : public Interval {
public:
    int vertex; // The vertex associated with the interval.

    /**
     * Constructor initializing the vertex and the interval with a start and end time.
     * @param vertex The vertex associated with the interval.
     * @param start The start time of the interval.
     * @param end The end time of the interval.
     */
    VInterval(int vertex, double start, double end) : Interval(start, end) {
        this->vertex = vertex;
    }

    /**
     * Constructor initializing the vertex and the interval with an existing interval.
     * @param vertex The vertex associated with the interval.
     * @param interval The existing interval.
     */
    VInterval(int vertex, Interval interval) : Interval(interval.start, interval.end) {
        this->vertex = vertex;
    }

    bool operator<(const VInterval& other) const {
        // is smaller if i end first
        if (end<other.end)
            return true;
        if (end > other.end)
            return false;
        // if we end at the same time, im smaller if i start first
        if (start < other.start)
            return true;
        if (start > other.start)
            return false;
        // if we start and end at the same time, im smaller if i am at a smaller vertex
        return vertex < other.vertex;
    }
};

/**
 * A class containing a method to bound the reward obtainable starting from a given node at a given time.
 */
class GraphBound {
public:
    Graph& graph; // The graph associated with this bound.
    Intervals intervals; // The intervals associated with this bound.

    Matrix min_dist; // The minimum distances between all pairs of nodes excluding half of the first and last edge.
    Matrix min_dist_no_last; // The minimum distances between all pairs of nodes excluding half of the last edge.

    map<VInterval, double> UBi; // a mapping from an interval to its corresponding UB_I value.

    /**
     * Constructor initializing the graph and intervals and computing all other variables.
     * @param graph The graph to initialize with.
     * @param intervals The intervals to initialize with.
     */
    GraphBound(Graph& graph, Intervals& intervals);

    /**
     * Function to calculate the upper bound for a given node at a given time [UB(u,t)].
     * @param u The node to calculate the upper bound for.
     * @param time The time at which to calculate the upper bound.
     * @return The calculated upper bound.
     */
    double UB(int u, double time);

private:
    /**
     * Function to compute the minimum distances between all pairs of nodes.
     */
    void computeDists();

    /**
     * Function to compute UB_I.
     */
    void computeUBI();
};

typedef set<TimeRewardPair> PairSet; // Represents a set of time-reward pairs.


/**
 * Wraps the GraphBound class to implement UB on a path.
 * Contains methods for caching rewards and determining whether a node should be pruned.
 */
class GraphBoundWrapper {
    GraphBound bound; // The GraphBound object to wrap.
    map<int, PairSet> cache; // A cache of UB(u,t) values

    /**
     * Function to add a reward for a given node at a given time to the cache.
     * @param u The node to add the reward for.
     * @param time The time at which to add the reward.
     * @param reward The reward to add.
     */
    void addToCache(int u, double time, double reward);

    /**
     * Function to retrieve a reward for a given node at a given time from the cache.
     * @param u The node to retrieve the reward for.
     * @param time The time at which to retrieve the reward.
     * @return A pair containing the retrieved reward and a boolean indicating whether the reward was found in the cache.
     */
    pair<double, bool> getFromCache(int u, double time);

public:
    /**
     * Constructor initializing the graph and intervals.
     * @param graph The graph to initialize with.
     * @param intervals The intervals to initialize with.
     */
    GraphBoundWrapper(Graph& graph, Intervals& intervals);

    /**
     * Function to determine whether a node should be pruned based on UB(pi).
     * @param u The node to check for pruning.
     * @param bound_pairs The list of bound pairs computed by the updatedOTP algorithm.
     * @param max_reward The maximum reward obtained until now.
     * @return True if the node should be pruned, false otherwise.
     */
    bool shouldPrune(int u, PairList& bound_pairs, double max_reward);

};


#endif //TASKASSISTANCE_BOUND_H
