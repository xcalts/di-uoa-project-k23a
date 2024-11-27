CXX = g++
CXXFLAGS = -std=c++14 -I./inc -I./libs

SRC_DIR = src
TESTS_DIR = tests
BIN_DIR = bin
SRC_OBJ_DIR = obj
TESTS_OBJ_DIR = test_obj

# Vamana Indexing
VAMANA_SRC = $(SRC_DIR)/main.cpp
VAMANA_OBJ = $(SRC_OBJ_DIR)/main.o

# Filtered Vamana Indexing
FILTERED_VAMANA_SRC = $(SRC_DIR)/newmain.cpp
FILTERED_VAMANA_OBJ = $(SRC_OBJ_DIR)/newmain.o

# Unit Tests
UNIT_TESTS_SRC = $(TESTS_DIR)/unitests.cpp
UNIT_TESTS_OBJ = $(TESTS_OBJ_DIR)/unitests.o

TARGETS = clean vamana unitests 

all: $(TARGETS)

# Compile the Vamana Indexing.
$(VAMANA_OBJ): $(VAMANA_SRC)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile the Filtered Vamana Indexing.
$(FILTERED_VAMANA_OBJ): $(FILTERED_VAMANA_SRC)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile the Unit Tests.
$(UNIT_TESTS_OBJ): $(UNIT_TESTS_SRC)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

vamana: $(BIN_DIR)/vamana
$(BIN_DIR)/vamana: $(VAMANA_OBJ)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(VAMANA_OBJ) -o $@

filtered_vamana: $(BIN_DIR)/filtered_vamana
$(BIN_DIR)/filtered_vamana: $(FILTERED_VAMANA_OBJ)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(FILTERED_VAMANA_OBJ) -o $@

unitests: $(BIN_DIR)/unitests
$(BIN_DIR)/unitests: $(UNIT_TESTS_OBJ) 
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(UNIT_TESTS_OBJ) -o $@ 

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O3 -march=native -ffast-math
release: all

clean:
	rm -rf $(SRC_OBJ_DIR) $(TESTS_OBJ_DIR) $(BIN_DIR) logs.txt

test: unitests
	$(BIN_DIR)/unitests

.PHONY: all clean vamana filtered_vamana unitests debug release