#############
# Variables #
#############
BIN_DIR = bin
CXX = g++
CXXFLAGS = -std=c++14 -I./inc -I./libs
TESTS_DIR = tests
SRC_DIR = src
SRC_OBJ_DIR = obj/src
TEST_OBJ_DIR = obj/tests
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
TESTS = $(wildcard $(TESTS_DIR)/*.cpp)
SRC_OBJS = $(filter-out $(SRC_OBJ_DIR)/main.o, $(patsubst $(SRC_DIR)/%.cpp, $(SRC_OBJ_DIR)/%.o, $(SRCS)))
TEST_OBJS = $(patsubst $(TESTS_DIR)/%.cpp, $(TEST_OBJ_DIR)/%.o, $(TESTS))
APP_OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(SRC_OBJ_DIR)/%.o, $(SRCS))
APP_TARGET = $(BIN_DIR)/app
TEST_TARGET = $(BIN_DIR)/unitests

#########
# Rules #
#########

all: $(APP_TARGET) $(TEST_TARGET)

# Compile application source files
$(SRC_OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Compile test files
$(TEST_OBJ_DIR)/%.o: $(TESTS_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Build application executable
$(APP_TARGET): $(APP_OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(APP_OBJS) -o $@

# Build test executable (exclude main.o from app sources)
$(TEST_TARGET): $(SRC_OBJS) $(TEST_OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(SRC_OBJS) $(TEST_OBJS) -o $@

# Debug build
debug: CXXFLAGS += -DDEBUG -g
debug: all

# Release build
release: CXXFLAGS += -O3 -march=native
release: all

# Clean build files
clean:
	rm -rf $(SRC_OBJ_DIR) $(TEST_OBJ_DIR) $(BIN_DIR) logs.txt

.PHONY: all clean app debug release
