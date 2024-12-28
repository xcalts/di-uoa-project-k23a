#############
# Variables #
#############
CXX = g++
CXXFLAGS = -std=c++14 -I./inc -I./libs
SRC_DIR = src
BIN_DIR = bin
SRC_OBJ_DIR = obj
TARGETS = app
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
OBJS = $(patsubst $(SRC_DIR)/%.cpp, $(SRC_OBJ_DIR)/%.o, $(SRCS))

#########
# Rules #
#########

all: $(TARGETS)

$(SRC_OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c $< -o $@

app: $(BIN_DIR)/app
$(BIN_DIR)/app: $(OBJS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@

debug: CXXFLAGS += -DDEBUG -g
debug: all

release: CXXFLAGS += -O3 -march=native
release: all

clean:
	rm -rf $(SRC_OBJ_DIR) $(BIN_DIR) logs.txt

.PHONY: all clean app debug release