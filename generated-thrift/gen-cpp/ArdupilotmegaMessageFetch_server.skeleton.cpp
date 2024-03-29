// This autogenerated skeleton file illustrates how to build a server.
// You should copy it to another filename to avoid overwriting it.

#include "ArdupilotmegaMessageFetch.h"
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;
using namespace ::apache::thrift::server;

using boost::shared_ptr;

using namespace  ::mavlink::thrift;

class ArdupilotmegaMessageFetchHandler : virtual public ArdupilotmegaMessageFetchIf {
 public:
  ArdupilotmegaMessageFetchHandler() {
    // Your initialization goes here
  }

  void availableMessages(std::map<ArdupilotmegaMessageTypes::type, int32_t> & _return) {
    // Your implementation goes here
    printf("availableMessages\n");
  }

  void fetchSensorOffsets(std::vector<SensorOffsets> & _return) {
    // Your implementation goes here
    printf("fetchSensorOffsets\n");
  }

  void fetchSetMagOffsets(std::vector<SetMagOffsets> & _return) {
    // Your implementation goes here
    printf("fetchSetMagOffsets\n");
  }

  void fetchMeminfo(std::vector<Meminfo> & _return) {
    // Your implementation goes here
    printf("fetchMeminfo\n");
  }

  void fetchApAdc(std::vector<ApAdc> & _return) {
    // Your implementation goes here
    printf("fetchApAdc\n");
  }

  void fetchDigicamConfigure(std::vector<DigicamConfigure> & _return) {
    // Your implementation goes here
    printf("fetchDigicamConfigure\n");
  }

  void fetchDigicamControl(std::vector<DigicamControl> & _return) {
    // Your implementation goes here
    printf("fetchDigicamControl\n");
  }

  void fetchMountConfigure(std::vector<MountConfigure> & _return) {
    // Your implementation goes here
    printf("fetchMountConfigure\n");
  }

  void fetchMountControl(std::vector<MountControl> & _return) {
    // Your implementation goes here
    printf("fetchMountControl\n");
  }

  void fetchMountStatus(std::vector<MountStatus> & _return) {
    // Your implementation goes here
    printf("fetchMountStatus\n");
  }

  void fetchFencePoint(std::vector<FencePoint> & _return) {
    // Your implementation goes here
    printf("fetchFencePoint\n");
  }

  void fetchFenceFetchPoint(std::vector<FenceFetchPoint> & _return) {
    // Your implementation goes here
    printf("fetchFenceFetchPoint\n");
  }

  void fetchFenceStatus(std::vector<FenceStatus> & _return) {
    // Your implementation goes here
    printf("fetchFenceStatus\n");
  }

  void fetchAhrs(std::vector<Ahrs> & _return) {
    // Your implementation goes here
    printf("fetchAhrs\n");
  }

  void fetchSimstate(std::vector<Simstate> & _return) {
    // Your implementation goes here
    printf("fetchSimstate\n");
  }

  void fetchHwstatus(std::vector<Hwstatus> & _return) {
    // Your implementation goes here
    printf("fetchHwstatus\n");
  }

  void fetchRadio(std::vector<Radio> & _return) {
    // Your implementation goes here
    printf("fetchRadio\n");
  }

  void fetchLimitsStatus(std::vector<LimitsStatus> & _return) {
    // Your implementation goes here
    printf("fetchLimitsStatus\n");
  }

  void fetchWind(std::vector<Wind> & _return) {
    // Your implementation goes here
    printf("fetchWind\n");
  }

  void fetchData16(std::vector<Data16> & _return) {
    // Your implementation goes here
    printf("fetchData16\n");
  }

  void fetchData32(std::vector<Data32> & _return) {
    // Your implementation goes here
    printf("fetchData32\n");
  }

  void fetchData64(std::vector<Data64> & _return) {
    // Your implementation goes here
    printf("fetchData64\n");
  }

  void fetchData96(std::vector<Data96> & _return) {
    // Your implementation goes here
    printf("fetchData96\n");
  }

  void fetchRangefinder(std::vector<Rangefinder> & _return) {
    // Your implementation goes here
    printf("fetchRangefinder\n");
  }

};

int main(int argc, char **argv) {
  int port = 9090;
  shared_ptr<ArdupilotmegaMessageFetchHandler> handler(new ArdupilotmegaMessageFetchHandler());
  shared_ptr<TProcessor> processor(new ArdupilotmegaMessageFetchProcessor(handler));
  shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
  shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
  shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

  TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);
  server.serve();
  return 0;
}

