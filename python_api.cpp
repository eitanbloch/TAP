#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "branch_and_bound.h"

namespace py = pybind11;

typedef const std::vector<std::vector<std::vector<double>>> PyGraph;
typedef const std::vector<std::vector<std::vector<double>>> PyIntervals;

class PyBranchAndBound {
public:
    PyBranchAndBound(Graph graph_,
                     Intervals intervals_,
                     double previous_max=0,
                     double prev_reward=0, 
                     double max_interval_size=1, 
                     double eps=0.02) : graph(graph_),
                                        intervals(intervals_),
                                        branchAndBound(graph, intervals, previous_max, eps, max_interval_size) {
    }
    

    double run() { return branchAndBound.run(); }
    Path getBestPath() { return branchAndBound.getBestPath(); }

private:
    Graph graph;
    Intervals intervals;
    BranchAndBound branchAndBound;


};


PYBIND11_MODULE(BranchAndBound, m) {
    py::class_<PyBranchAndBound>(m, "BranchAndBound")
        .def(py::init<Graph&, Intervals&, double, double, double>())
        .def("run", &PyBranchAndBound::run)
        .def("getBestPath", &PyBranchAndBound::getBestPath);
    py::class_<AdjEntry>(m, "AdjEntry")
        .def(py::init<int, double>())
        .def_readwrite("v", &AdjEntry::v)
        .def_readwrite("w", &AdjEntry::w);
    py::class_<Interval>(m, "Interval")
        .def(py::init<double, double>())
        .def_readwrite("start", &Interval::start)
        .def_readwrite("end", &Interval::end);
    py::class_<Edge>(m, "Edge")
        .def(py::init<int, int, double>())
        .def_readwrite("u", &Edge::u)
        .def_readwrite("v", &Edge::v)
        .def_readwrite("w", &Edge::w);
}