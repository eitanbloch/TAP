#include "branch_and_bound.h"

BranchAndBound::BranchAndBound(Graph& graph, Intervals& intervals, double previous_max, double eps,
                               double max_interval_size) : OPTP(graph, intervals),
                                                           bound(GraphBoundWrapper(graph, intervals)),
                                                           value_history(graph.size()), eps(eps),
                                                           max_interval_size(max_interval_size),
                                                           dummy_vertex((int) graph.size()) {

    // update max value
    max_value = previous_max;

    // preprocessing
    preprocess_intervals();
    preprocess_graph();

    // add intervals for dummy vertex
    intervals.emplace_back();
}

void BranchAndBound::preprocess_intervals() {
    // if interval length is more than max_interval_size, split it
    for (auto& vec: intervals) {
        IntervalList temp_list;
        for (auto& interval: vec) {
            auto start = interval.start, end = interval.end;
            while (end - start > max_interval_size) {
                temp_list.push_back({Interval(start, start + max_interval_size)});
                start += max_interval_size;
            }
            temp_list.push_back({Interval(start, end)});
        }
        vec = temp_list;
    }
}

void BranchAndBound::preprocess_graph() {
    // sort adj list from longest edge to shortest
    for (auto& vec: graph) {
        sort(vec.begin(), vec.end(), [this](const AdjEntry& a, const AdjEntry& b) {
            if (intervals[a.v].size() == intervals[b.v].size())
                return a.w > b.w;
            return intervals[a.v].size() > intervals[b.v].size();
        });
    }
}

pair<double, PairList *> BranchAndBound::runOTP(Path& path, double w_min) {
    path_count++;
    // add dummy edge to the end of the path
    int last_vertex = path.empty() ? START_VERTEX : path.back().v;
    path.emplace_back(last_vertex, dummy_vertex, w_min);
    intervals.emplace_back();

    // run OTP
    auto res = updatedOTP(path, intervals);

    // remove dummy edge
    path.pop_back();
    intervals.pop_back();

    return res;
}


double BranchAndBound::run() {
    // start timer
    start_time = std::chrono::high_resolution_clock::now();

    auto path = Path();
    branchAndBound(path, 0);

    return max_value;
}

void BranchAndBound::branchAndBound(Path& path, double path_length) {
    // check if time limit is reached
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration =
            (double) std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.;
    if (duration > time_limit) {
        cout << "OPTP: Time limit reached" << endl;
        return;
    }

    int last_vertex = path.empty() ? -1 : path.back().u;
    int current_vertex = path.empty() ? START_VERTEX : path.back().v;

    // min edge length from current vertex (this can be added to the path when running OTP)
    double w_min = graph[current_vertex].empty() ? 0 : graph[current_vertex].back().w;

    // run OTP
    auto pair = runOTP(path, w_min);
    double value = pair.first;
    PairList *bound_pairs = pair.second;

    // prune unnecessary loops using ValueHistory to keep track of
    if (value_history.last(current_vertex) > value - EPS) {
        return;
    }

    // update real value
    value_history.add(current_vertex, value);

    // update max reward
    if (value > max_value) {
        max_value = value;
        max_path = path;
    }

    // iterate over all adjacent edges
    for (auto& adj: graph[current_vertex]) {
        // if vertex is not reachable
        if (path_length + adj.w > 1)
            continue;

        // prune unnecessary loops
        if (adj.v == last_vertex && intervals[adj.v].empty())
            continue;

        // prune if upper bound is less than max value
        if (bound.shouldPrune(adj.v, *bound_pairs, (1 + eps) * max_value))
            continue;

        // add edge and run recursive call
        path.emplace_back(current_vertex, adj.v, adj.w);
        branchAndBound(path, path_length + adj.w);
        path.pop_back();
    }

    // update value history
    value_history.removeLast(current_vertex);

    delete bound_pairs;
}
