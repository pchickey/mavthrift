
#ifndef __CALCULATOR_MGHANDLER_H__
#define __CALCULATOR_MGHANDLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "mongoose.h"

int calculator_mghandler(struct mg_connection *conn,
                         const struct mg_request_info *request_info);

#ifdef __cplusplus
}
#endif
#endif // __CALCULATOR_MGHANDLER_H__

