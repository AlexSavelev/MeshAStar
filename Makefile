.DEFAULT_GOAL := mesh_astar
CC = g++  
ERR =  -Wall -Werror -Wextra
OPT1 = -O3
CFLAGS = -std=c++11 $(ERR) $(OPT1)

OBJECTS = obj/KC_kinematic.o obj/KC_heuristic.o obj/KC_heap.o obj/KC_searching.o obj/KC_structs.o obj/KC_testing.o obj/KC_search_params.o obj/KC_algorithms.o
HEADERS = $(wildcard include/*.hpp)

OUTPUT = mesh_astar

$(OBJECTS): obj/%.o  : src/%.cpp $(HEADERS) 
	$(CC) $(CFLAGS) -I include -c  $< -o $@

$(OBJECTS): | obj

$(OUTPUT): obj $(OBJECTS)
	$(CC) $(CFLAGS) -I include -o $@ $(OBJECTS)

obj: 
	mkdir -p obj
	mkdir -p res

clean:
	rm -rf obj
	rm -rf $(OUTPUT)

.PHONY: test
test: obj/heuristic_tests obj/kinematic_tests obj/additive_tests obj/search_structure_tests obj/directional_tests obj/path_cost_tests obj/grid_bucket_tests
	./obj/heuristic_tests
	./obj/kinematic_tests
	./obj/additive_tests
	./obj/search_structure_tests
	./obj/directional_tests
	./obj/path_cost_tests
	./obj/grid_bucket_tests

obj/grid_bucket_tests: tests/grid_bucket_tests.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/path_cost_tests: tests/path_cost_tests.cpp include/KC_path_cost.hpp | obj
	$(CC) $(CFLAGS) -I include -o $@ $<

obj/heuristic_tests: tests/heuristic_tests.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

.PHONY: wall-experiment
wall-experiment: obj/wall_experiment

obj/wall_experiment: tools/wall_experiment.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/kinematic_tests: tests/kinematic_tests.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

.PHONY: kinematic-experiment
kinematic-experiment: obj/kinematic_experiment

obj/kinematic_experiment: tools/kinematic_experiment.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

.PHONY: multimap-experiment
multimap-experiment: obj/multimap_experiment

obj/multimap_experiment: tools/multimap_experiment.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/diagnose_multimap: tools/diagnose_multimap.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/inspect_cases: tools/inspect_cases.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/additive_tests: tests/additive_tests.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/additive_experiment: tools/additive_experiment.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/diagnose_additive: tools/diagnose_additive.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/alpha_experiment: tools/alpha_experiment.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/alpha_bounds: tools/alpha_bounds.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/search_structure_tests: tests/search_structure_tests.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/search_structure_experiment: tools/search_structure_experiment.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/directional_pilot: tools/directional_pilot.cpp tools/directional_potential.hpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/directional_tests: tests/directional_tests.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/directional_experiment: tools/directional_experiment.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/directional_holdout: tools/directional_holdout.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))

obj/inspect_directional_cases: tools/inspect_directional_cases.cpp $(filter-out obj/KC_testing.o,$(OBJECTS)) $(HEADERS) | obj
	$(CC) $(CFLAGS) -I include -o $@ $< $(filter-out obj/KC_testing.o,$(OBJECTS))
