
#include "SerialReader.h"

#include "mavlink-c/ardupilotmega/mavlink.h"

using namespace mavlink::thrift;

void SerialReader::registerHandler(SimpleFetchServerHandler &h) {
    _handlers.push_back(h);
}

void run() {

}

