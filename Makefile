OUT = physics 
CXX = g++
CXXFLAGS = -g -Wall -std=c++11
INCLUDE = -I$(IMGUI_DIR)
LDLIBS = 
SRC_DIR = src/
OBJ_DIR = obj/
CXXFILES = $(wildcard $(SRC_DIR)*.cc)
OFILES = $(patsubst $(SRC_DIR)%.cc, $(OBJ_DIR)%.o, $(CXXFILES))
CONFIG_MSG = "Configuration: Release"
UNAME_S := $(shell uname -s)

# ImGui library files
IMGUI_DIR = imgui-docking/
IMGUI_SRC = $(wildcard $(IMGUI_DIR)imgui*.cpp)
#IMGUI_OBJ = $(addsuffix .o, $(basename $(notdir $(IMGUI_SRC))))
IMGUI_OBJ = $(patsubst $(IMGUI_DIR)%.cpp, $(OBJ_DIR)%.o, $(IMGUI_SRC))

ifeq ($(UNAME_S), Linux)
	PLATFORM_MSG = "Linux"
	INCLUDE += `pkg-config --cflags sdl2`
	LDLIBS = `pkg-config --libs sdl2` -lSDL2_image -lSDL2_ttf
endif

ifeq ($(OS), Windows_NT)
	PLATFORM_MSG = "MinGW"
	INCLUDE += `pkg-config --cflags sdl2_ttf sdl2_image`
	LDLIBS = `pkg-config --libs sdl2 sdl2_ttf sdl2_image` -mconsole
endif

#------------------------
# BUILD RULES
#------------------------

all: setup $(OUT)
	@printf "%b\n" "\033[32mBuild complete\033[0m for $(PLATFORM_MSG)"
	@echo $(CONFIG_MSG)

$(OUT): $(OFILES) $(IMGUI_OBJ)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $^ -o $@ $(LDLIBS)
	@echo "------------------------"

$(OBJ_DIR)%.o: $(IMGUI_DIR)%.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

$(OBJ_DIR)%.o: $(SRC_DIR)%.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

#-------------------------
# CUSTOM RULES
#-------------------------

.PHONY: debug
debug: CXXFLAGS += -DDEBUG -O0
debug: CONFIG_MSG = "Configuration: Debug"
debug: all

.PHONY: setup
setup:
	@mkdir -p $(OBJ_DIR)

.PHONY: run
run: all
	./$(OUT)

.PHONY: clean
clean:
	@rm -f $(OUT) $(OFILES) $(OFILES:.o=.d)
	
.PHONY: remake
remake: clean all

#-------------------------
# DEPENDENCIES GENERATION
#-------------------------

CXXFLAGS += -MMD
-include $(OFILES:.o=.d)
