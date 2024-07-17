#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// #include <pybind11/complex.h>
// #include <pybind11/functional.h>
// #include <pybind11/chrono.h>
#include "branch_and_bound.h"

namespace py = pybind11;

typedef const std::vector<std::vector<std::vector<double>>> PyGraph;
typedef const std::vector<std::vector<std::vector<double>>> PyIntervals;


float branchAndBound(PyGraph pyGraph,
                     PyIntervals pyIntervals,
                     double prev_reward=0, double max_size=0.01, double eps=0.02) {
    
    // pyGraph is a list of vertices, each holds a list of descendants ([index, edge_weight])
    // pyIntervals is a list of vertices, each holds a list of descendants ([index, edge_weight])
    
    Graph graph;
    for (const auto& pyVertex : pyGraph) {
        vector<AdjEntry> descendants_vector;
        for (const auto& pyDescendant : pyVertex) {
            descendants_vector.push_back(AdjEntry(int(pyDescendant[0]), double(pyDescendant[1])));
        }
        graph.push_back(descendants_vector);
    }

    Intervals intervals;
    for (const auto& pyVertex : pyIntervals) {
        IntervalList intervalList;
        for (const auto& interval : pyVertex) {
            intervalList.push_back(Interval(double(interval[0]), double(interval[1])));
        }
        intervals.push_back(intervalList);
    }

    BranchAndBound branchAndBound = BranchAndBound(graph, intervals, eps, max_size);
    double coverage = branchAndBound.run();
    return coverage;
}

PYBIND11_MODULE(BranchAndBound, m) {
    m.def("branchAndBound", &branchAndBound, "branch and bound function");
}









/*
Python example:


import pickle
from BranchAndBound import branchAndBound
from otp import find_intervals

def create_path_and_intervals(planner):
    plan_length = planner.max_plan_length
    plan, _, _, _, _, _, indices = planner.get_best_plan()
    path = [[indices[i], indices[i+1], planner.search_tree.time_to_reach_configuration(plan[i], plan[i+1])/plan_length] for i in range(len(indices) - 1)]
    coverages = {indices[i]: planner.search_tree.calc_configuration_coverage(plan[i], 0) for i in range(len(indices))}
    intervals_dict = {index: find_intervals(coverage)/plan_length for index, coverage in coverages.items()}
    graph = []
    intervals = []
    vertices_map = {}
    
    for edge in path:
        vertices_map[len(graph)] = edge[0]
        graph.append([[edge[1], edge[2]]])
        intervals.append([[interval[0], interval[1]] for interval in intervals_dict[edge[0]]])
    vertices_map[len(graph)] = path[-1][1]
    intervals.append([[interval[0], interval[1]] for interval in intervals_dict[path[-1][1]]])
    graph.append([])

    inverse_vertices_map = {v: k for k, v in vertices_map.items()}
    for i in range(len(graph)):
        if len(graph[i]) == 0:
            continue
        graph[i][0][0] = inverse_vertices_map[graph[i][0][0]]

    return graph, intervals, vertices_map

with open("/home/dor/thesis/outputs/13-07-2024_21:05:35/planner.pickle", "rb") as f:
    planner = pickle.load(f)
graph, intervals, vertices_map = create_path_and_intervals(planner)

branchAndBound(graph, intervals, 0, 0.01, 0.02)

*/





// #include <pybind11/pybind11.h>
// #include <pybind11/stl.h>
// #include <vector>

// namespace py = pybind11;

// float sum_2d_list(const std::vector<std::vector<float>>& input) {
//     float sum = 0;
//     for (const auto& row : input) {
//         for (const auto& val : row) {
//             sum += val;
//         }
//     }
//     return sum;
// }

// PYBIND11_MODULE(example, m) {
//     m.def("sum_2d_list", &sum_2d_list, "A function that sums elements of a 2D list");
// }
