OUT = physics 
CXX = g++
CXXFLAGS = -g -Wall -std=c++11
INCLUDE = `pkg-config --cflags sdl2 sdl2_ttf sdl2_image`
LDLIBS = `pkg-config --libs sdl2 sdl2_ttf sdl2_image` -mconsole
CXXFILES = main.cc system_state.cc rigid_body.cc collision.cc \
		   scene_editor.cc render.cc utils.cc link.cc vector2.cc
OFILES = $(CXXFILES:.cc=.o)

all: $(OUT)

$(OUT): $(OFILES)
	$(CXX) $(CXXFLAGS) $(INCLUDE) $(OFILES) -o $@ $(LDLIBS)

%.o: %.cc
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@ $(INCLUDE)

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
main.o: main.cc system_state.h render.h rigid_body.h vector2.h \
 collision.h link.h scene_editor.h utils.h
system_state.o: system_state.cc system_state.h render.h rigid_body.h \
 vector2.h collision.h link.h utils.h config.h
rigid_body.o: rigid_body.cc rigid_body.h render.h vector2.h collision.h \
 config.h
collision.o: collision.cc collision.h rigid_body.h render.h vector2.h \
 config.h
render.o: render.cc render.h
utils.o: utils.cc utils.h
link.o: link.cc link.h rigid_body.h render.h vector2.h
vector2.o: vector2.cc vector2.h
