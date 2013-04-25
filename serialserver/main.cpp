
#include <stdio.h>
#include <stdint.h>

#include "ArdupilotmegaMessagePost.h"
#include "ArdupilotmegaMessageFetch.h"
#include <thrift/protocol/TBinaryProtocol.h>
#include <thrift/server/TSimpleServer.h>
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TServerSocket.h>
#include <thrift/transport/TBufferTransports.h>

#include "SimpleFetchServer.h"

using namespace boost;
using namespace apache::thrift;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;
using namespace apache::thrift::server;

using namespace mavlink::thrift;


static int port = 9090;

int fetchServer(void) {
  shared_ptr<SimpleFetchServerHandler> handler(new SimpleFetchServerHandler());
  shared_ptr<TProcessor> processor(new ArdupilotmegaMessageFetchProcessor(handler));
  shared_ptr<TServerTransport> serverTransport(new TServerSocket(port));
  shared_ptr<TTransportFactory> transportFactory(new TBufferedTransportFactory());
  shared_ptr<TProtocolFactory> protocolFactory(new TBinaryProtocolFactory());

  TSimpleServer server(processor, serverTransport, transportFactory, protocolFactory);

  Heartbeat hb;
  hb.mavtype = 1; hb.autopilot = 13; hb.mavlink_version = 3;
  handler->postHeartbeat(hb);

  server.serve();
  return 0;
}

int fetchClient (void) {
    shared_ptr<TSocket> socket(new TSocket("localhost", port));
    shared_ptr<TTransport> transport(new TBufferedTransport(socket));
    shared_ptr<TProtocol> protocol(new TBinaryProtocol(transport));

    ArdupilotmegaMessageFetchClient client(protocol);
    std::map<ArdupilotmegaMessageTypes::type, int32_t> available;

    getchar();
    transport->open();
    client.availableMessages(available);
    std::vector<Heartbeat> hbmessages;
    client.fetchHeartbeat(hbmessages);

    transport->close();

    return 0;
}

int main(int argc, char* argv[]) {
    if (argc > 1) {
        fetchServer();
    } else {
        fetchClient();
    }
    return 0;
}

