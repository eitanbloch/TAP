#ifndef TASKASSISTANCE_DFS_OTP_H
#define TASKASSISTANCE_DFS_OTP_H

#include "optp.h"


/**
 * A DFS-based OPTP solver iterating over all possible paths in a DFS like approach and running the OTP algorithm on each path.
 * Note that this is identical to Branch and Bound OPTP with a trivial bound (always returning 1).
 */
class DFSOTP: public OPTP{

    /**
     * Runs the OTP algorithm on a path.
     *
     * @param path The path to run the OTP algorithm on.
     * @return The maximum reward obtained by the OTP solver.
     */
    double runOTP(Path& path);

    /**
     * Runs the DFS based OTP solver on a path.
     *
     * @param path The path to start the DFSOTP solver from.
     * @param path_length The length of the path.
     */
    void dfsOTP(Path &path, double path_length);

public:
    /**
     * Constructs a new DFSOTP object.
     *
     * @param graph The graph to solve the OTP for.
     * @param intervals The intervals for each vertex in the graph.
     */
    DFSOTP(Graph& graph, Intervals& intervals);

    /**
     * Runs the DFSOTP solver.
     *
     * @return The maximum reward obtained by the DFSOTP solver.
     */
    double run() override;
};

#endif //TASKASSISTANCE_DFS_OTP_H