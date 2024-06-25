#ifndef TASKASSISTANCE_DELTA_DISCRETIZATION_H
#define TASKASSISTANCE_DELTA_DISCRETIZATION_H

#include "optp.h"

/**
 * A Delta Discretization solver for the OPTP problem.
 */
class DeltaDiscretization: public OPTP {
    double delta; // The delta value for the Delta Discretization solver.

    /**
     * Runs the Delta Discretization algorithm given a path.
     *
     * @param path The path to run the Delta Discretization algorithm from.
     * @param path_length The length of the path.
     * @return The maximum reward obtained by the Delta Discretization solver.
     */
    double delta_discretization(Path& path, double path_length);

public:
    /**
     * Constructs a new DeltaDiscretization object.
     *
     * @param graph The graph to solve the OPTP for.
     * @param intervals The intervals for each vertex in the graph.
     * @param delta The delta value for the Delta Discretization solver.
     */
    DeltaDiscretization(Graph& graph, Intervals& intervals, double delta);

    /**
     * Runs the Delta Discretization solver.
     *
     * @return The maximum reward obtained by the Delta Discretization solver.
     */
    double run() override;

};

#endif //TASKASSISTANCE_DELTA_DISCRETIZATION_H