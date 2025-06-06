#pragma once 

#include <cstdint>

namespace canopen {
namespace sdo {

enum Type {
    // request
    REQ_WRITE_4BYTES = 0x23,
    REQ_WRITE_3BYTES = 0x27,
    REQ_WRITE_2BYTES = 0x2B,
    REQ_WRITE_1BYTE  = 0x2F,
    REQ_READ = 0x40,

    // response
    RESP_WRITE_NORMAL = 0x60,
    RESP_READ_4BYTES = 0x43,
    RESP_READ_3BYTES = 0x47,
    RESP_READ_2BYTES = 0x4B,
    RESP_READ_1BYTE  = 0x4F,
    RESP_EXCEPTION = 0x80,
};

}

}