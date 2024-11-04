CXX = g++
CXXFLAGS = -std=c++14 -I./inc -I./libs

BIN_DIR = bin

SRC_DIR = src
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_DIR = obj
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SOURCES))

# Exclude main.o for unit tests
OBJECTS_NO_MAIN = $(filter-out $(OBJ_DIR)/main.o, $(OBJECTS))

TESTS_SRC_DIR = tests
TEST_SOURCES = $(wildcard $(TESTS_SRC_DIR)/*.cpp)
T_OBJ_DIR = test_obj
TEST_OBJECTS = $(patsubst $(TESTS_SRC_DIR)/%.cpp, $(T_OBJ_DIR)/%.o, $(TEST_SOURCES))

TARGETS = k23a unitests

all: $(TARGETS)

# Compile test source files into object files
$(T_OBJ_DIR)/%.o: $(TESTS_SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile main source files into object files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

k23a: $(BIN_DIR)/k23a

$(BIN_DIR)/k23a: $(OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $@

unitests: $(BIN_DIR)/unitests

# Link unit tests executable without main.o
$(BIN_DIR)/unitests: $(OBJECTS_NO_MAIN) $(TEST_OBJECTS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(OBJECTS_NO_MAIN) $(TEST_OBJECTS) -o $@

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O3 -march=native -ffast-math
release: all

clean:
	rm -rf $(OBJ_DIR) $(T_OBJ_DIR) $(BIN_DIR) logs.txt

.PHONY: all clean k23a unitests debug release
