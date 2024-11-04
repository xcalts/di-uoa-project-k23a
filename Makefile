# Makefile

CXX = g++
CXXFLAGS = -std=c++14 -I./inc -I./libs

SRC_DIR = src
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)

OBJ_DIR = obj
BIN_DIR = bin
OBJECTS = $(patsubst $(SRC_DIR)/%.cpp, $(OBJ_DIR)/%.o, $(SOURCES))
TEST_OBJECTS = $(patsubst $(TESTS_SRC_DIR)/%.cpp, $(TEST_OBJ_DIR)/%.o, $(TEST_SOURCES))

TESTS_SRC_DIR = tests
TEST_SOURCES = $(wildcard $(TESTS_SRC_DIR)/*.cpp)

TARGETS = clean build k23a unitests


all: $(TARGETS)

# rule to compile the test .cpp files into .o files
$(OBJ_DIR)/%.o: $(TESTS_SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@


# rule to compile .cpp files into .o files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# rule to build the program
k23a: $(OBJECTS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(BIN_DIR)/$@

# rule to build unit tests
unitests: $(OBJECTS) $(TEST_OBJECTS)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(OBJECTS) $(TEST_OBJECTS) -o $(BIN_DIR)/$@


# rule for debug
debug: CXXFLAGS += -DDEBUG -g
debug: all

# rule for release
release: CXXFLAGS += -O3 -march=native -ffast-math
release: all

build:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR) logs.txt

.PHONY: all clean
