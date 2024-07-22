from BranchAndBound import BranchAndBound, AdjEntry, Interval

# Naive graph (a path):
path = [{"u": 0, "v": 1, "w": 0.02582},
         {"u": 1, "v": 2, "w": 0.01829},
         {"u": 2, "v": 3, "w": 0.01815},
         {"u": 3, "v": 4, "w": 0.02252},
         {"u": 4, "v": 5, "w": 0.01724},
         {"u": 5, "v": 6, "w": 0.02286},
         {"u": 6, "v": 7, "w": 0.01203},
         {"u": 7, "v": 8, "w": 0.02542},
         {"u": 8, "v": 9, "w": 0.02351},
         {"u": 9, "v": 10, "w": 0.00741},
         {"u": 10,"v": 11, "w": 0.01176}]

# Dictionary of all intervals for each node
intervals_dict = {0: [],
                  1: [[0.21183801, 0.2305296 ]],
                  2: [[0.20560748, 0.23987539]],
                  3: [[0.2211838 , 0.23987539]],
                  4: [[0.08722741, 0.11214953], [0.18380062, 0.21806854], [0.29906542, 0.30218069]],
                  5: [[0.20872274, 0.23987539], [0.29906542, 0.30529595]],
                  6: [[0.29283489, 0.29283489], [0.35825545, 0.38629283], [0.46417445, 0.47040498], [0.6728972 , 0.67912773], [0.78193146, 0.81931464], [0.92211838, 0.92834891]],
                  7: [[0.34267913, 0.38006231]],
                  8: [[0.34267913, 0.36760125]],
                  9: [[0.47975078, 0.74454829], [0.85669782, 1.        ]],
                  10: [[0.47352025, 0.76012461], [0.8411215 , 1.        ]],
                  11: [[0.43302181, 1.        ]]}


#creating graph using the C++ objects
graph = []
intervals = []
for edge in path:
    graph.append([AdjEntry(edge["v"], edge["w"])])
    intervals.append([Interval(interval[0], interval[1]) for interval in intervals_dict[edge["u"]]])
intervals.append([Interval(interval[0], interval[1]) for interval in intervals_dict[path[-1]["v"]]])
graph.append([])

# Running the Branch and Bound
bab = BranchAndBound(graph, intervals, 0, 0.01, 0.02)
bab.run()
bab.getBestPath()