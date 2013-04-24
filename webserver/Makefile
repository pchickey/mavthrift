
CFLAGS = -I../mongoose/ -Wall
CXXFLAGS = -fexceptions -I../mongoose/ -I./thrift/ -Wall -DHAVE_INTTYPES_H -DHAVE_NETINET_IN_H
LDFLAGS = -lpthread -ldl


THRIFTOBJS = $(addprefix thrift/,Calculator.o \
	shared_constants.o \
	shared_types.o \
	SharedService.o \
	tutorial_constants.o \
	tutorial_types.o \
	)

OBJS = mongoose.o main.o calculator_mghandler.o $(THRIFTOBJS)

default: server

mongoose.o: ../mongoose/mongoose.c
	$(CC) $(CFLAGS) -c ../mongoose/mongoose.c -o mongoose.o 

server: $(OBJS)
	$(CXX) -o server $(LDFLAGS) $(OBJS) libthrift.a

clean:
	-rm *.o
	-rm thrift/*.o
	-rm server
