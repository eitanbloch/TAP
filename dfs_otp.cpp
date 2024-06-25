#include "dfs_otp.h"

DFSOTP::DFSOTP(Graph& graph, Intervals& intervals) : OPTP(graph, intervals) {}


double DFSOTP::runOTP(Path& path) {
    path_count++;

    if (path.empty())
        return 0;

    return OTP(path, intervals);
}


void DFSOTP::dfsOTP(Path& path, double path_length) {
    // check if time limit is reached
    auto current_time = std::chrono::high_resolution_clock::now();
    auto duration =
            (double) std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count() / 1000.;
    if (duration > time_limit) {
        cout << "DFS_OPTP: Time limit reached" << endl;
        return;
    }

    if (path_length > 1)
        return;

    // run otp on current path
    auto res = runOTP(path);
    if (res > max_value) {
        max_value = res;
        max_path = path;
    }

    // iterate over all adjacent vertices
    auto last_vertex = path.empty() ? START_VERTEX : path.back().v;
    for (auto& adj: graph[last_vertex]) {
        path.emplace_back(last_vertex, adj.v, adj.w);
        dfsOTP(path, path_length + adj.w);
        path.pop_back();
    }
}

double DFSOTP::run() {
    start_time = std::chrono::high_resolution_clock::now();

    auto path = Path();
    dfsOTP(path, 0);

    return max_value;
}

