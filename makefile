MODE ?= default

ifeq ($(MODE), default)
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

else ifeq ($(MODE), py)
# Define the Python interpreter that also contains pybind11 package installed
PYTHON = /home/dor/miniconda3/envs/TIP_env_3.10/bin/python

# Get the necessary compiler and linker flags for pybind11
PYBIND11_INCLUDE = $(shell $(PYTHON) -m pybind11 --includes)
PYTHON_EXTENSION_SUFFIX = $(shell $(PYTHON) -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))")

# Define the C++ compiler and flags
CXX = g++
CXXFLAGS = -O3 -Wall -shared -std=c++11 -fPIC


# The name of the shared library to be created
TARGET = BranchAndBound$(PYTHON_EXTENSION_SUFFIX)

# The source file
SRC = python_api.cpp branch_and_bound.cpp otp.cpp bound.cpp

# The rule to build the shared library
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(PYBIND11_INCLUDE) $(SRC) -o $(TARGET)

# Clean up the build
clean:
	rm -f $(TARGET)
endif