#ifndef TASKASSISTANCE_OTP_H
#define TASKASSISTANCE_OTP_H

#include <iostream>
#include <cmath>
#include "utils.h"

/**
 * Computes the reward for a given vertex and entry and exit times [R(u,t,t')].
 *
 * @param intervals The list of intervals belonging to the vertex (I(u)).
 * @param entry_time The entry time (t).
 * @param exit_time The exit time (t').
 * @return The computed reward.
 */
double computeReward(IntervalList& intervals, double entry_time, double exit_time);

/**
 * Computes the distance to the end of a path.
 *
 * @param path The path.
 * @return The computed distance.
 */
double computeDistanceToEnd(Path& path);

/**
 * Computes the CT0 set for a given path and intervals.
 *
 * @param path The path.
 * @param intervals The intervals.
 * @return A pointer to the computed TimeSet.
 */
TimeSet *computeCT0(Path& path, Intervals& intervals);

/**
 * Computes the reward of the optimal timing profile (OTP) for a given path and intervals.
 *
 * @param path The path.
 * @param intervals The intervals.
 * @return The reward of an optimal timing profile.
 */
double OTP(Path& path, Intervals& intervals);

/**
 * Updated OTP version for the OPTP problem
 *
 * @param path The path.
 * @param intervals The intervals.
 * @return A pair containing the reward of an optimal timing profile, and the PairList needed to compute the upper bound of a path.
 */
pair<double, PairList *> updatedOTP(Path& path, Intervals& intervals);

#endif //TASKASSISTANCE_OTP_H