
#include <stdio.h>
#include <stdint.h>

#include "ArdupilotmegaMessagePostService.h"
#include "ArdupilotmegaMessageFetchService.h"
#include <thrift/transport/TSocket.h>
#include <thrift/transport/TBufferTransports.h>
#include <thrift/protocol/TBinaryProtocol.h>

using namespace boost;
using namespace apache::thrift;
using namespace apache::thrift::protocol;
using namespace apache::thrift::transport;

using namespace mavlink::thrift;

int main (void) {
    shared_ptr<TSocket> socket(new TSocket("localhost", 9090));
    shared_ptr<TTransport> transport(new TBufferedTransport(socket));
    shared_ptr<TProtocol> protocol(new TBinaryProtocol(transport));

    ArdupilotmegaMessageFetchServiceClient client(protocol);
    std::map<ArdupilotmegaMessageTypes::type, int32_t> available;

    getchar();
    transport->open();
    client.availableMessages(available);
    transport->close();

    return 0;
}
