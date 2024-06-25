#include "utils.h"
#include "branch_and_bound.h"
#include "dfs_otp.h"
#include "delta_discretization.h"

/**
 * Run the Branch and Bound algorithm.
 * @param filename The name of the file containing the graph and intervals.
 * @param previous_max The maximum value obtained on the previous graph.
 * @param eps The epsilon approximation value.
 * @param max_interval_size The maximum interval size.
 * @return The maximum reward obtained by the Branch and Bound solver.
 */
double runBranchAndBound(string& filename, double prev_reward, double max_size, double eps) {
    auto data = loadData(filename);
    string file = filename.substr(filename.find_last_of('/') + 1);

    auto start = chrono::high_resolution_clock::now();
    auto alg = BranchAndBound(data.first, data.second, prev_reward, eps, max_size);
    double reward = alg.run();
    auto end = chrono::high_resolution_clock::now();

    cout << "[B&B] File: " << file << ": Time: "
         << chrono::duration_cast<chrono::milliseconds>(end - start).count() / 1000. << "s, Reward: " << reward << endl;

    return reward;
}

/**
 * Run the DFS-OTP algorithm.
 * @param filename The name of the file containing the graph and intervals.
 * @return The maximum reward obtained by the DFS-OTP solver.
 */
double runDFSOTP(string& filename) {
    auto data = loadData(filename);
    string file = filename.substr(filename.find_last_of('/') + 1);

    auto start = chrono::high_resolution_clock::now();
    auto alg = DFSOTP(data.first, data.second);
    double reward = alg.run();
    auto end = chrono::high_resolution_clock::now();

    cout << "[DFS-OTP] File: " << file << ": Time: "
         << chrono::duration_cast<chrono::milliseconds>(end - start).count() / 1000. << "s, Reward: " << reward << endl;

    return reward;
}
/**
 * Run the Delta Discretization algorithm.
 * @param filename The name of the file containing the graph and intervals.
 * @param delta The delta value for the Delta Discretization solver.
 * @return The maximum reward obtained by the Delta Discretization solver.
 */
double runDeltaDiscretization(string& filename, double delta) {
    auto data = loadData(filename);
    string file = filename.substr(filename.find_last_of('/') + 1);

    auto start = chrono::high_resolution_clock::now();
    auto alg = DeltaDiscretization(data.first, data.second, delta);
    double reward = alg.run();
    auto end = chrono::high_resolution_clock::now();

    cout << "[DD] File: " << file << ": Time: "
         << chrono::duration_cast<chrono::milliseconds>(end - start).count() / 1000. << "s, Reward: " << reward << endl;

    return reward;
}

/**
 * Run the Branch and Bound algorithm on a roadmap.
 * @param dirname The name of the directory containing the graph files.
 * @param N The maximal size to run the Branch and Bound algorithm on.
 * @param max_size The maximum interval size for the Branch and Bound solver.
 * @param eps The epsilon approximation value for the Branch and Bound solver.
 */
void continuousBranchAndBound(string& dirname, double N, double max_size, double eps) {
    double reward = 0;
    for (int i = 10; i <= N; i+=10) {
        string filename = dirname + "/graph_" + to_string(i) + ".txt";
        reward = runBranchAndBound(filename, reward, max_size, eps);
    }
}

int main() {
    double prev_reward, max_size, eps, delta, N;
    string filename = "test_files/graph2/graph_10.txt";

    // run Branch and Bound
    prev_reward = 0;
    max_size = 0.01;
    eps = 0.02;
    runBranchAndBound(filename, prev_reward, max_size, eps);

    // run Delta Discretization
    delta = 0.2;
    runDeltaDiscretization(filename, delta);

    // run DFS-OTP
    runDFSOTP(filename);

    cout << "---------------------------------------" << endl;

    // run continuous Branch and Bound with parameters: N=1000, max_size=0.01, eps=0.02
    string dirname = "test_files/graph3";
    N = 1000;
    max_size = 0.01;
    eps = 0.02;
    continuousBranchAndBound(dirname, N, max_size, eps);

    return 0;
}
