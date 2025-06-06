#pragma once

#include "Message.h"
#include "Instructions.h"
#include <memory>

namespace canopen {
namespace sdo {

class Response : public Message {
public:
    // for response
    Response(uint32_t idCan, uint8_t bytes[8]) : Message(idCan, bytes) {}

    inline uint32_t GetIns() {
        return index << 8 | subIndex;
    }
};

}
}