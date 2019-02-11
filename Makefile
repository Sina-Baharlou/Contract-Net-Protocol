CXXFLAGS =	-std=c++0x -O2 -g -Wall -fmessage-length=0 -Wreorder -Wwrite-strings -Wsign-compare

OBJS =		CNP.o DAgent.o ObstacleManager.o PolicyManager.o VAgent.o

LIBS =		-lopencv_core -lpthread -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs

TARGET =	CNP

LIBSDIR = 	

$(TARGET):	$(OBJS)
	$(CXX) $(LIBSDIR) -o $(TARGET) $(OBJS) $(LIBS)

all:	$(TARGET)

clean:
	rm -f $(OBJS) $(TARGET)
