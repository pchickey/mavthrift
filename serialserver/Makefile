
CFLAGS = -Wall

CXXFLAGS = -Wall -g3
CXXFLAGS += -I../generated-thrift/gen-cpp/ 
CXXFLAGS += -I../
CXXFLAGS += -DHAVE_INTTYPES_H -DHAVE_NETINET_IN_H

LDFLAGS = -L/usr/local/lib -Wl,-rpath,/usr/local/lib

LIBS = ../generated-thrift/libmavthrift.a -lthrift -lpthread -ldl

OBJS = main.o SimpleFetchServer.o SerialReader.o
default: serialserver

serialserver: $(OBJS)
	$(CXX) $(LDFLAGS) -o serialserver $(OBJS) $(LIBS)

clean:
	-rm *.o
	-rm serialserver
