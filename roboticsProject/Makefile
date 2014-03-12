OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

CXXINCLUDE+=-I/usr/local/include

LXXLIBS+=-L/usr/local/lib
LXXLIBS+=-lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lSDL -lpython2.7


flameDetection: all
	$(CXX) $(CXXDEFINE) -o $@  $(OBJS)  $(CXXFLAGS) $(LXXLIBS) 

all: $(OBJS) 

.cpp.o:
	$(CXX)  $(CXXFLAGS) $(CXXDEFINE) -c  $(CXXINCLUDE) $< 

.c.o:
	$(CXX)  $(FLAGS) $(CXXDEFINE) -c  $(CXXFLAGS) $(CXXINCLUDE) $< 

clean:
	$(RM) $(OBJS) *.moc $(UI_HEAD) $(UI_CPP)
