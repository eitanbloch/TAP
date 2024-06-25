#include "delta_discretization.h"


DeltaDiscretization::DeltaDiscretization(Graph& graph, Intervals& intervals, double delta) : OPTP(graph, intervals), delta(delta) {}

double DeltaDiscretization::run() {
    auto path = Path();
    start_time = std::chrono::high_resolution_clock::now();
    delta_discretization(path, 0);
    return max_value;
}



double DeltaDiscretization::delta_discretization(Path& path, double path_length) {
    // check if time limit is reached
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration =
            (double) std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.;

    if (duration > time_limit) {
        cout << "Delta Discretization: Time limit reached" << endl;
        return 0;
    }

    if (path_length > 1)
        return 0;

    double local_max_value = 0;
    Path local_max_path;

    auto last_vertex = path.empty() ? START_VERTEX : path.back().v;

    // option 1 - wait for epsilon
    auto reward = computeReward(intervals[last_vertex], path_length, path_length + delta) +
                  delta_discretization(path, path_length + delta);
    if (reward > local_max_value) {
        local_max_value = reward;
        local_max_path = path;
    }

    // option 2 - move to an adjacent vertex
    for (auto& adj: graph[last_vertex]) {
        path.emplace_back(last_vertex, adj.v, adj.w);
        // reward obtained during the edge
        auto edge_reward = computeReward(intervals[last_vertex], path_length, path_length + adj.w / 2)
                           + computeReward(intervals[adj.v], path_length + adj.w / 2, path_length + max(adj.w, delta));
        // total reward
        reward = edge_reward + delta_discretization(path, path_length + max(adj.w, delta));

        if (reward > local_max_value) {
            local_max_value = reward;
            local_max_path = path;
        }
        path.pop_back();
    }

    if (local_max_value > max_value) {
        max_value = local_max_value;
        max_path = local_max_path;
    }

    return local_max_value;
}
