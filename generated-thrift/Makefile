
MAVTHRIFT_DIR      =.
MAVTHRIFT_CPP_DIR  =$(MAVTHRIFT_DIR)/gen-cpp

MAVTHRIFT_CPPS = $(filter-out $(wildcard $(MAVTHRIFT_CPP_DIR)/*.skeleton.cpp),\
			$(wildcard $(MAVTHRIFT_CPP_DIR)/*.cpp))
MAVTHRIFT_HS   = $(wildcard $(MAVTHRIFT_CPP_DIR)/*.h)
MAVTHRIFT_OBJS = $(MAVTHRIFT_CPPS:.cpp=.o)

THRIFT_CXXFLAGS = -DHAVE_INTTYPES_H -DHAVE_NETINET_IN_H

CXXFLAGS += -Wall -g3

%.o : CXXFLAGS += $(THRIFT_CXXFLAGS)
%.o : %.cpp
	$(CXX) -c $(CXXFLAGS) -o $@ $^

libmavthrift.a: $(MAVTHRIFT_OBJS)
	$(AR) rcs $@ $^

clean:
	-rm gen-cpp/*.o
	-rm libmavthrift.a
