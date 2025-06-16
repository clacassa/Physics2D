OUT = physics 
CXX = g++
CXXFLAGS = -g -Wall -std=c++17 -O3
INCLUDE = -I$(IMGUI_DIR)
LDLIBS = 
SRC_DIR = src/
OBJ_DIR = obj/
CXXFILES = $(wildcard $(SRC_DIR)*.cc)
OFILES = $(patsubst $(SRC_DIR)%.cc, $(OBJ_DIR)%.o, $(CXXFILES))
CONFIG_MSG = "Configuration: Release"
UNAME_S := $(shell uname -s)

# ImGui library files
IMGUI_DIR = imgui/
IMGUI_SRC = $(wildcard $(IMGUI_DIR)imgui*.cpp)
#IMGUI_OBJ = $(addsuffix .o, $(basename $(notdir $(IMGUI_SRC))))
IMGUI_OBJ = $(patsubst $(IMGUI_DIR)%.cpp, $(OBJ_DIR)%.o, $(IMGUI_SRC))

ifeq ($(UNAME_S), Linux)
	PLATFORM_MSG = "Linux"
	INCLUDE += `pkg-config --cflags sdl2`
	LDLIBS = `pkg-config --libs sdl2`
endif

ifeq ($(OS), Windows_NT)
	PLATFORM_MSG = "MinGW"
	INCLUDE += `pkg-config --cflags sdl2`
	LDLIBS = `pkg-config --libs sdl2` -lSDL2_gfx -mconsole
endif

#------------------------
# BUILD RULES
#------------------------

all: setup $(OUT)
	@printf "%b\n" "\033[32mBuild complete\033[0m for $(PLATFORM_MSG)"
	@echo $(CONFIG_MSG)

$(OUT): $(OFILES) $(IMGUI_OBJ) $(TRACY_OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $^ -o $@ $(LDLIBS)
	@echo "------------------------"

$(OBJ_DIR)%.o: $(IMGUI_DIR)%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)%.o: $(SRC_DIR)%.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

$(TRACY_OBJ): $(TRACY_SRC)
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

#-------------------------
# CUSTOM RULES
#-------------------------

.PHONY: debug
debug: CXXFLAGS += -DDEBUG -O0
debug: CONFIG_MSG = "Configuration: Debug"
debug: all

# Tracy profiler
.PHONY: tracy
tracy: TRACY_DIR = tracy-0.11.1/public/
tracy: TRACY_SRC = $(TRACY_DIR)TracyClient.cpp
tracy: TRACY_OBJ = $(OBJ_DIR)TracyClient.o
tracy: CXXFLAGS += -DTRACY_ENABLE
tracy: INCLUDE += -I$(TRACY_DIR)tracy/
ifeq ($(OS), Windows_NT)
tracy: LDLIBS += -lws2_32 -lwsock32 -ldbghelp
endif
tracy: CONFIG_MSG += " (Tracy profiling enabled)"
tracy: all

.PHONY: setup
setup:
	@mkdir -p $(OBJ_DIR)

.PHONY: run
run: all
	./$(OUT)

.PHONY: clean
clean:
	@rm -f $(OUT) $(OFILES) $(OFILES:.o=.d) $(TRACY_OBJ)
	
.PHONY: purge
purge:
	@rm -f $(OUT) $(OFILES) $(OFILES:.o=.d) $(IMGUI_OBJ) $(TRACY_OBJ)

.PHONY: remake
remake: clean all

#-------------------------
# DEPENDENCIES GENERATION
#-------------------------

CXXFLAGS += -MMD
-include $(OFILES:.o=.d)
