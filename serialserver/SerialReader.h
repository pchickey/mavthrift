
#ifndef SerialReader_H
#define SerialReader_H

#include "SimpleFetchServer.h"

class SerialReader {
  public:
    SerialReader(const char* fname) : _fname(fname) {}
    void registerHandler(mavlink::thrift::SimpleFetchServerHandler &h);
    void run();
  private:
    const char* _fname;
    std::vector<mavlink::thrift::SimpleFetchServerHandler> _handlers;
};

#endif // SerialReader_H
