OUT = physics 
CXX = g++
CXXFLAGS = -g -Wall -std=c++11
INCLUDE =
LDLIBS = 
SRC_DIR = src
SRC_FILES = main.cc control.cc system_state.cc rigid_body.cc collision.cc \
		   editor.cc render.cc utils.cc link.cc vector2.cc
CXXFILES = $(SRC_FILES:%=$(SRC_DIR)/%)
OFILES = $(SRC_FILES:.cc=.o)
UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S), Linux)
	ECHO_MESSAGE = "Linux"
	INCLUDE += `pkg-config --cflags sdl2`
	LDLIBS = `pkg-config --libs sdl2` -lSDL2_image -lSDL2_ttf
endif

ifeq ($(OS), Windows_NT)
	ECHO_MESSAGE = "MinGW"
	INCLUDE += `pkg-config --cflags sdl2_ttf sdl2_image`
	LDLIBS = `pkg-config --libs sdl2 sdl2_ttf sdl2_image` -mconsole
endif

#------------------------
# BUILD RULES
#------------------------

all: $(OUT)
	@echo "-----------------------------"
	@echo "Build complete for" $(ECHO_MESSAGE)

$(OUT): $(OFILES)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(OFILES) -o $@ $(LDLIBS)

%.o: $(SRC_DIR)/%.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

#-------------------------
# CUSTOM RULES
#-------------------------

.PHONY: run
run: $(OUT)
	./$(OUT)

.PHONY: clean
clean:
	@rm -f $(OUT) $(OFILES)

.PHONY: depend
depend:
	@echo " *** UPDATING DEPENDENCIES ***"
	@(sed '/^# DO NOT DELETE THIS LINE/q' Makefile && \
		$(CXX) -MM $(CXXFLAGS) $(CXXFILES) | \
		egrep -v "/usr/include" \
	) > Makefile.new
	@mv Makefile.new Makefile

#
#
#
# DO NOT DELETE THIS LINE
main.o: src/main.cc src/control.h src/vector2.h src/system_state.h \
 src/render.h src/rigid_body.h src/collision.h src/link.h src/editor.h \
 src/utils.h
control.o: src/control.cc src/control.h src/vector2.h
system_state.o: src/system_state.cc src/system_state.h src/render.h \
 src/rigid_body.h src/vector2.h src/collision.h src/link.h src/utils.h \
 src/config.h
rigid_body.o: src/rigid_body.cc src/rigid_body.h src/render.h \
 src/vector2.h src/collision.h src/utils.h src/config.h
collision.o: src/collision.cc src/collision.h src/rigid_body.h \
 src/render.h src/vector2.h src/config.h
editor.o: src/editor.cc src/editor.h src/vector2.h src/render.h \
 src/utils.h
render.o: src/render.cc src/render.h
utils.o: src/utils.cc src/utils.h src/render.h
link.o: src/link.cc src/link.h src/rigid_body.h src/render.h \
 src/vector2.h
vector2.o: src/vector2.cc src/vector2.h
