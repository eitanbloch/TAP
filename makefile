CXX = g++
CXXFLAGS = -std=c++11 -O3

OBJS = example.o utils.o bound.o otp.o branch_and_bound.o dfs_otp.o delta_discretization.o

all: clean TAP

TAP: $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^
	rm -f $(OBJS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f *.o executable

.PHONY: all clean