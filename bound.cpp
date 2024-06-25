#include "bound.h"

void dijkstra(Graph& graph, int start, vector<double>& dist) {
    set<pair<double, int>> s;
    s.insert({0, start});
    dist[start] = 0;
    while (!s.empty()) {
        auto it = s.begin();
        int u = it->second;
        s.erase(it);
        for (auto& adj: graph[u]) {
            int v = adj.v;
            double w = adj.w;
            if (dist[v] > dist[u] + w) {
                s.erase({dist[v], v});
                dist[v] = dist[u] + w;
                s.insert({dist[v], v});
            }
        }
    }
}


GraphBound::GraphBound(Graph& graph, Intervals& intervals) : graph(graph), intervals(intervals) {
    min_dist.resize(graph.size(), vector<double>(graph.size(), INF));
    min_dist_no_last.resize(graph.size(), vector<double>(graph.size(), INF));
    computeDists();
    computeUBI();
}

void GraphBound::computeDists() {
    // real distances
    Matrix distances = Matrix(graph.size(), vector<double>(graph.size(), INF));
    for (int u = 0; u < graph.size(); u++) {
        dijkstra(graph, u, distances[u]);
    }

    // distances without half of last edge
    min_dist_no_last = distances;
    for (int s = 0; s < graph.size(); s++) {
        for (int u = 0; u < graph.size(); u++) {
            for (auto& edge: graph[u]) {
                int v = edge.v;
                double w = edge.w;
                min_dist_no_last[s][v] = min(min_dist_no_last[s][v], distances[s][u] + w / 2);
            }
        }
    }

    // distances without half of first and last edge
    min_dist = min_dist_no_last;
    for (int u = 0; u < graph.size(); u++) {
        for (auto& edge: graph[u]) {
            for (int t = 0; t < graph.size(); t++) {
                int v = edge.v;
                double w = edge.w;
                min_dist[u][t] = min(min_dist[u][t], min_dist_no_last[v][t] + w / 2);
            }
        }
    }

    // set distances to 0 for all edges
    for (int u = 0; u < graph.size(); u++) {
        for (auto edge: graph[u])
            min_dist[u][edge.v] = 0;
    }
}

void GraphBound::computeUBI() {
    // create list of (interval, vertex) pairs
    vector<VInterval> v_intervals;
    for (int u = 0; u < intervals.size(); u++) {
        for (auto& interval: intervals[u]) {
            v_intervals.emplace_back(u, interval);
        }
    }
    // sort by interval.end from largest to smallest
    sort(v_intervals.begin(), v_intervals.end(),
         [](const VInterval& a, const VInterval& b) { return a.end > b.end; }
    );

    // initialize UBi
    for (auto v_interval: v_intervals) {
        UBi[v_interval] = v_interval.end - v_interval.start;
    }

    // iterate over all intervals
    bool changed = true; // flag to check if we need to iterate again
    while (changed) {
        changed = false;

        for (auto v_interval: v_intervals) {
            for (auto next_interval: v_intervals) {
                // consider only intervals that are after v_interval
                if (v_interval.start >= next_interval.end - EPS) {
                    continue;
                }

                auto dist = min_dist[v_interval.vertex][next_interval.vertex];
                auto exit_time = min(max(v_interval.start, next_interval.start - dist), v_interval.end);
                auto arrival_time = exit_time + dist;

                auto reward_in_interval = exit_time - v_interval.start;
                auto future_reward = UBi[next_interval] - max(0.0, arrival_time - next_interval.start);
                auto reward = reward_in_interval + future_reward;
                if (reward > UBi[v_interval] + EPS) {
                    UBi[v_interval] = reward;
                    changed = true;
                }
            }
        }
    }
}

double GraphBound::UB(int u, double time) {
    double bound = 0;
    // iterate over v_intervals
    for (auto pair: UBi) {
        auto v_interval = pair.first;
        auto reward = pair.second;

        auto distance = min_dist_no_last[u][v_interval.vertex];
        if (time + distance < v_interval.end + EPS) {
            auto overlap = max(time + distance - v_interval.start, 0.0);
            bound = max(bound, reward - overlap);
        }
    }

    return bound;
}


GraphBoundWrapper::GraphBoundWrapper(Graph& graph, Intervals& intervals) : bound(graph, intervals) {
    // initialize cache
    for (int u = 0; u < graph.size(); u++) {
        cache[u] = PairSet();
    }
}

void GraphBoundWrapper::addToCache(int u, double time, double reward) {
    cache[u].insert({time, reward});
}

pair<double, bool> GraphBoundWrapper::getFromCache(int u, double time) {
    // if cache is empty, return false
    if (cache[u].empty())
        return {1, false};

    // find the closest time that is less than or equal to time
    auto it = cache[u].lower_bound({time, 0});
    if (it == cache[u].begin())
        return {1, false};
    it--;
    return {it->reward, it->time == time};
}


bool GraphBoundWrapper::shouldPrune(int u, PairList& bound_pairs, double max_reward) {
    // iterate over all exit times
    for (auto& pair: bound_pairs) {
        auto exit_time = pair.time;
        auto reward = pair.reward;

        auto entry = getFromCache(u, exit_time);
        if (reward + entry.first <= max_reward)
            continue;
        // if entry is accurate, we cant prune
        if (entry.second) {
            return false;
        }

        // compute bound
        auto real_bound = bound.UB(u, exit_time);
        addToCache(u, exit_time, real_bound);
        if (reward + real_bound > max_reward) {
            return false;
        }
    }

    return true;
}
