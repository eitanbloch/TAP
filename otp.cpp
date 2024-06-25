#include "otp.h"

double computeReward(IntervalList& intervals, double entry_time, double exit_time) {
    double reward = 0;

    for (auto& interval: intervals) {
        if (exit_time < interval.start)
            break;
        if (entry_time > interval.end)
            continue;

        // compute intersection
        double start = max(entry_time, interval.start);
        double end = min(exit_time, interval.end);
        reward += end - start;
    }

    return reward;
}

double computeDistanceToEnd(Path& path) {
    double distance = 0;
    for (auto& edge: path) {
        distance += edge.w;
    }
    distance -= path.back().w / 2;
    return distance;
}

TimeSet *computeCT0(Path& path, Intervals& intervals) {
    auto *CT = new TimeSet();
    double time_to_subtract = -path.front().w / 2; // equal to delta+(v_0, v_i)

    for (auto& edge: path) {
        // add first half of edge
        time_to_subtract += edge.w / 2.0;

        // add intervals for edge
        for (auto& interval: intervals[edge.u]) {
            auto exit_time = interval.end - time_to_subtract;
            if (exit_time >= 0)
                CT->insert(exit_time);
        }

        for (auto& interval: intervals[edge.v]) {
            auto exit_time = interval.start - time_to_subtract;
            if (exit_time >= 0)
                CT->insert(exit_time);
        }

        // add second half of edge
        time_to_subtract += edge.w / 2.0;
    }

    return CT;
}

double OTP(Path& path, Intervals& intervals) {
    // add dummy edge to the end of the path
    int new_vertex = (int) intervals.size();
    path.emplace_back(path.back().v, new_vertex, 0);

    // add dummy intervals
    double first_edge = path.front().w;
    intervals[path.front().u].insert(intervals[path.front().u].begin(), {first_edge / 2, first_edge / 2});
    intervals.push_back({{1, 1}});

    // critical times
    auto *CT0 = computeCT0(path, intervals);

    // Entry and Exit lists
    auto Entry = new PairList(), Exit = new PairList();

    // initialize Entry list
    Entry->push_back({0, 0});

    double dist_to_vertex = -path.front().w / 2, last_edge_weight = 0;

    for (auto& edge: path) {
        double min_time_at_vertex = (last_edge_weight + edge.w) / 2.0;
        dist_to_vertex += min_time_at_vertex;
        last_edge_weight = edge.w;

        int next_start = 0; // index of next entry time to consider (optimization)

        for (auto exit_time: *CT0) {
            exit_time += dist_to_vertex; // convert to CTi

            // iterate over all entry times
            double max_reward = 0;
            for (int i = next_start; i < Entry->size(); i++) {
                auto& entry = Entry->at(i);
                if (entry.time + min_time_at_vertex > exit_time + EPS) // entry time is too late for exit time
                    break;

                double reward = entry.reward + computeReward(intervals[edge.u], entry.time, exit_time);
                if (reward > max_reward + EPS) {
                    max_reward = reward;
                    next_start = i;
                }
            }

            // Pareto optimization
            if (Exit->empty() || max_reward > Exit->back().reward + EPS)
                Exit->push_back({exit_time, max_reward}); // add to exit list
        }

        // update entry and exit list
        delete Entry;
        Entry = Exit;
        Exit = new PairList();
    }

    double optimal_reward = Entry->back().reward;

    // remove dummies
    path.pop_back();
    intervals[path.front().u].erase(intervals[path.front().u].begin());
    intervals.pop_back();

    // clean up
    delete Entry;
    delete Exit;
    delete CT0;

    return optimal_reward;
}

pair<double, PairList *> updatedOTP(Path& path, Intervals& intervals) {
    // no need to add dummy edge

    // add dummy intervals
    double first_edge = path.front().w;
    intervals[path.front().u].insert(intervals[path.front().u].begin(), {first_edge / 2, first_edge / 2});
    intervals[path.back().v].emplace_back(1, 1);

    // critical times
    auto *CT0 = computeCT0(path, intervals);

    // Entry and Exit lists
    auto Entry = new PairList(), Exit = new PairList();

    // initialize Entry list
    Entry->push_back({0, 0});

    // pairs needed to compute the upper bound of a path
    auto BoundPairs = new PairList();

    double dist_to_vertex = -path.front().w / 2, last_edge_weight = 0;
    double dist_to_end = computeDistanceToEnd(path); // distance from start to end
    double direct_reward = 0; // !

    for (auto& edge: path) {
        double min_time_at_vertex = (last_edge_weight + edge.w) / 2.0;
        dist_to_vertex += min_time_at_vertex;
        last_edge_weight = edge.w;

        // bound related variables
        auto current_intervals = intervals[edge.u];
        int bound_interval_index = 0;
        // ignore unreachable intervals
        while (bound_interval_index < current_intervals.size() &&
               current_intervals[bound_interval_index].end < dist_to_vertex)
            bound_interval_index++;

        int next_start = 0; // index of next entry time to consider (optimization)

        for (auto exit_time: *CT0) {
            exit_time += dist_to_vertex; // convert to CTi

            // iterate over all entry times
            double max_reward = 0, max_reward2 = 0;
            for (int i = next_start; i < Entry->size(); i++) {
                auto& entry = Entry->at(i);
                if (entry.time + min_time_at_vertex > exit_time + EPS) // entry time is too late for exit time
                    break;

                double reward = entry.reward + computeReward(intervals[edge.u], entry.time, exit_time);

                if (reward > max_reward + EPS) {
                    max_reward = reward;
                    next_start = i;
                }
            }

            // Pareto optimization
            if (Exit->empty() || max_reward > Exit->back().reward + EPS)
                Exit->push_back({exit_time, max_reward}); // add to exit list


            // if exit time corresponds to the end of the next interval
            if (bound_interval_index < current_intervals.size() &&
                fabs(exit_time - current_intervals[bound_interval_index].end) < EPS) {

                auto dist_left = dist_to_end - dist_to_vertex; // distance from current vertex to end
                auto time_at_end = current_intervals[bound_interval_index].start + dist_left; // corresponding time at the end of the path
                bound_interval_index++;
                if (time_at_end > 1)
                    continue;

                // iterate over all entry times
                max_reward = 0;
                for (int i = next_start; i < Entry->size(); i++) {
                    auto& entry = Entry->at(i);
                    if (entry.time > exit_time + EPS) // computed without adding min_time_at_vertex (see footnote at Appendix C of the paper)
                        break;

                    double reward = entry.reward + computeReward(intervals[edge.u], entry.time, exit_time);
                    if (reward > max_reward + EPS) {
                        max_reward = reward;
                        next_start = i;
                    }
                }
                BoundPairs->push_back({time_at_end, max_reward});
            }
            // special case: exit time corresponds to the earliest time the current vertex can be left
            if (fabs(exit_time - dist_to_vertex) < EPS) {
                // iterate over all possible entry times
                max_reward = 0;

                for (int i = next_start; i < Entry->size(); i++) {
                    auto& entry = Entry->at(i);
                    if (entry.time > exit_time + EPS)
                        break;

                    double reward = entry.reward + computeReward(intervals[edge.u], entry.time, exit_time);
                    if (reward > max_reward + EPS) {
                        max_reward = reward;
                        next_start = i;
                    }
                }

                direct_reward = max_reward;
            }
        }
        
        // update entry and exit list
        delete Entry;
        Entry = Exit;
        Exit = new PairList();
    }

    double optimal_reward = Entry->back().reward;

    // push pair from the special case
    BoundPairs->push_back({dist_to_vertex, direct_reward});

    // remove dummies
    intervals[path.front().u].erase(intervals[path.front().u].begin());
    intervals[path.back().v].pop_back();

    // clean up
    delete Entry;
    delete Exit;
    delete CT0;

    return {optimal_reward, BoundPairs};
}

