
#include <stdio.h>
#include <string.h>
#include "mongoose.h"

#include "calculator_mghandler.h"

// This function will be called by mongoose on every new request.
static int begin_request_handler(struct mg_connection *conn) {
    const struct mg_request_info *request_info = mg_get_request_info(conn);
    char content[100];

    if (strcmp(request_info->uri, "/") == 0) {
        // Prepare the message we're going to send
        int content_length = snprintf(content, sizeof(content),
                                      "Hello from mongoose! Remote port: %d",
                                      request_info->remote_port);

        // Send HTTP reply to the client
        mg_printf(conn,
                  "HTTP/1.1 200 OK\r\n"
                  "Content-Type: text/plain\r\n"
                  "Content-Length: %d\r\n"        // Always set Content-Length
                  "\r\n"
                  "%s",
                  content_length, content);

        // Returning non-zero tells mongoose that our function has replied to
        // the client, and mongoose should not send client any more data.
        return 1;
    } else if (strcmp(request_info->uri, "/thrift/service/tutorial/") == 0) {
        int ret = calculator_mghandler(conn, request_info);
        return ret;
    } else {
        return 0;
    }
}

int main(void) {
  struct mg_context *ctx;
  struct mg_callbacks callbacks;

  // List of options. Last element must be NULL.
  const char *options[] = {"listening_ports", "8080",
                           "document_root", "./webroot",
                           NULL};

  // Prepare callbacks structure. We have only one callback, the rest are NULL.
  memset(&callbacks, 0, sizeof(callbacks));
  callbacks.begin_request = begin_request_handler;

  // Start the web server.
  ctx = mg_start(&callbacks, NULL, options);

  // Wait until user hits "enter". Server is running in separate thread.
  // Navigating to http://localhost:8080 will invoke begin_request_handler().
  getchar();

  // Stop the server.
  mg_stop(ctx);

  return 0;
}
