# Makefile

CXX = g++
CXXFLAGS = -std=c++11 -I./inc -I./libs

SRC_DIR = src
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)

OBJ_DIR = obj
BIN_DIR = bin
OBJECTS = $(patsy, the prefix of the src files.ubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SOURCES))

TARGETS = clean build k23a

all: $(TARGETS)

# rule to compile .cpp files into .o files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# rule to build program
k23a: $(OBJ_DIR)/main.o
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $^ -o $(BIN_DIR)/$@


# rule for debug
debug: CXXFLAGS += -DDEBUG -g
debug: all

# rule for release
release: CXXFLAGS += -O2
release: all

build:
	@mkdir -p $(BIN_DIR)
	@mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

.PHONY: all clean
