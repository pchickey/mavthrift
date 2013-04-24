
#include <stdint.h>

#include "calculator_mghandler.h"

#include "Calculator.h"
#include "SharedService.h"

#include <thrift/protocol/TJSONProtocol.h>
#include <thrift/transport/TBufferTransports.h>

using namespace ::apache::thrift;
using namespace ::apache::thrift::protocol;
using namespace ::apache::thrift::transport;

using boost::shared_ptr;

using namespace ::tutorial;
using namespace ::shared;


class CalculatorHandler : virtual public CalculatorIf {
 public:
  CalculatorHandler() {}

  void ping() {
    printf("[CH] ping\n");
  }

  int32_t add(const int32_t num1, const int32_t num2) {
    printf("[CH] add %d %d\n", num1, num2);
    return num1 + num2;
  }

  int32_t calculate(const int32_t logid, const Work& work) {
    printf("[CH] calculate %d %d %d %d\n",
            logid, work.op, work.num1, work.num2);

    int32_t val;

    switch(work.op) {
        case Operation::ADD:
            val = work.num1 + work.num2;
            break;
        case Operation::SUBTRACT:
            val = work.num1 - work.num2;
            break;
        case Operation::MULTIPLY:
            val = work.num1 * work.num2;
            break;
        case Operation::DIVIDE:
            if (work.num2 == 0) {
                InvalidOperation io;
                io.what = work.op;
                io.why = "illegal div by 0";
                throw io;
            }
            val = work.num1 / work.num2;
            break;
        default:
            InvalidOperation io;
            io.what = work.op;
            io.why = "Invalid Operation";
            throw io;
    }
    return val;
  }

  void getStruct(SharedStruct &ret, int32_t logid) {
    printf("[CH] getStruct unimplemented!\n");
  }
  void zip() {
    printf("[CH] zip\n");
  }

};

int calculator_mghandler(struct mg_connection *conn,
                         const struct mg_request_info *request_info) {
    try {
        shared_ptr<CalculatorHandler> handler(new CalculatorHandler());
        shared_ptr<TProcessor> processor(new CalculatorProcessor(handler));

        uint8_t intermediate[64];
        int bytes_read;
        shared_ptr<TMemoryBuffer> input(new TMemoryBuffer());
        shared_ptr<TJSONProtocol> input_json(new TJSONProtocol(input));

        do {
            bytes_read = mg_read(conn, intermediate, sizeof(intermediate));
            input->write(intermediate, bytes_read);
        } while (bytes_read > 0);

        shared_ptr<TMemoryBuffer> output(new TMemoryBuffer());
        shared_ptr<TJSONProtocol> output_json(new TJSONProtocol(output));

        bool success = processor->process(input_json, output_json, NULL);

        if (success) {
            do {
                bytes_read = output->read(intermediate, sizeof(intermediate));
                mg_write(conn, intermediate, bytes_read);
            } while (bytes_read > 0);
        }
    } catch (TProtocolException *te) {
        printf("calculator_msghandler: TProtocolException %s\n");
    } catch (TException *e) {
        printf("calculator_msghandler: TException\n");
    } catch (...) {
        printf("calculator_msghandler: unknown exception\n");
    }

    return 1;
}
