# Standalone portable build; no binaries copied from another machine.
CXX ?= c++
CXXFLAGS ?= -std=c++11 -O3 -Wall -Wextra -Werror
BENCH_SRC = src/KC_kinematic.cpp src/KC_heuristic.cpp src/KC_heap.cpp src/KC_searching.cpp src/KC_structs.cpp src/KC_search_params.cpp src/KC_algorithms.cpp
BENCH_OBJ = $(patsubst src/%.cpp,build-benchmark/%.o,$(BENCH_SRC))
BENCH_HEADERS = $(wildcard include/*.hpp)
.PHONY: all
all: build-benchmark/benchmark_worker
build-benchmark:
	mkdir -p $@
build-benchmark/%.o: src/%.cpp $(BENCH_HEADERS) | build-benchmark
	$(CXX) $(CXXFLAGS) -Iinclude -c $< -o $@
build-benchmark/benchmark_worker: tools/benchmark_worker.cpp $(BENCH_OBJ) $(BENCH_HEADERS)
	$(CXX) $(CXXFLAGS) -Iinclude $< $(BENCH_OBJ) -o $@
