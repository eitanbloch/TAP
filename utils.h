#ifndef TASKASSISTANCE_UTILS_H
#define TASKASSISTANCE_UTILS_H

#include <iostream>
#include <vector>
#include <set>

using namespace std;

// The start vertex in a graph.
#define START_VERTEX 0

// A small value for comparison purposes.
const double EPS = 1e-6;

/**
 * Represents an interval with a start and end time.
 */
class Interval {
public:
    double start; // The start time of the interval.
    double end; // The end time of the interval.

    /**
     * Constructs a new Interval object.
     *
     * @param start The start time of the interval.
     * @param end The end time of the interval.
     */
    Interval(double start, double end) : start(start), end(end) {}
};

/**
 * Represents a pair of time and reward.
 */
class TimeRewardPair {
public:
    double time; // The time component of the pair.
    double reward; // The reward component of the pair.

    /**
     * Constructs a new TimeRewardPair object.
     *
     * @param time The time component of the pair.
     * @param reward The reward component of the pair.
     */
    TimeRewardPair(double time, double reward) : time(time), reward(reward) {}

    /**
     * Constructs a new TimeRewardPair object from a pair of doubles.
     *
     * @param p The pair of doubles to construct the TimeRewardPair from.
     */
    TimeRewardPair(const pair<double, double>& p) : time(p.first), reward(p.second) {} // TODO: check usage

    bool operator==(const TimeRewardPair& other) const {
        return time == other.time && reward == other.reward;
    }

    bool operator<(const TimeRewardPair& other) const {
        if (time < other.time)
            return true;
        if (time > other.time)
            return false;
        return reward < other.reward;
    }
};

/**
 * Represents an edge in a graph as two vertices and a weight.
 */
class Edge {
public:
    int u; // The first vertex of the edge.
    int v; // The second vertex of the edge.
    double w; // The weight of the edge.

    /**
     * Constructs a new Edge object.
     *
     * @param u The first vertex of the edge.
     * @param v The second vertex of the edge.
     * @param w The weight of the edge.
     */
    Edge(int u, int v, double w) : u(u), v(v), w(w) {}
};

/**
 * Represents an adjacency list entry with a vertex and a weight.
 */
class AdjEntry {
public:
    int v; // The vertex of the adjacency list entry.
    double w; // The weight of the adjacency list entry.

    /**
     * Constructs a new AdjEntry object.
     *
     * @param v The vertex of the adjacency list entry.
     * @param w The weight of the adjacency list entry.
     */
    AdjEntry(int v, double w) : v(v), w(w) {}
};

// Represents a path as a vector of edges.
typedef vector<Edge> Path;

// Represents a graph as an adjacency list.
typedef vector<vector<AdjEntry>> Graph;

// Represents a list of intervals.
typedef vector<Interval> IntervalList;

// Represents a mapping of vertices to sequences of intervals.
typedef vector<IntervalList> Intervals;

// Represents a set of times.
typedef set<double> TimeSet;

// Represents a list of time-reward pairs.
typedef vector<TimeRewardPair> PairList;

/**
 * Loads a graph and its corresponding interval's mapping from a given filename
 *
 * @param filename The name of the file to load the data from.
 * @return A pair containing the loaded graph and intervals.
 */
pair<Graph, Intervals> loadData(const string& filename);

#endif //TASKASSISTANCE_UTILS_H