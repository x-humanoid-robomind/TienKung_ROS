#pragma once

#include "Message.h"
#include "Instructions.h"

namespace canopen {
namespace sdo {

class Request : public Message {
public:
    Request(uint32_t idCob, ReqIns req, uint32_t data = 0) :
        Message(idCob, Type(req >> 24& 0x00FF) , uint32_t(req >> 8 & 0x00FFFF), uint32_t(req & 0x00FF), data) {}
};

}
}