#pragma once

#include <vector>
#include <cstdint>
#include <glog/logging.h>

#include "Type.h"

namespace canopen {
namespace sdo {

class Message {
public:

    uint32_t idCob;
    uint32_t idCan;
    Type type;
    uint32_t index;
    uint32_t subIndex;
    uint32_t data;  // data or about code

    Message() = default;

    // for response
    Message(uint32_t idCan, uint8_t bytes[8]) {
        idCob = idCan - 0x580;
        type = (Type)bytes[0];
        index = bytes[2] << 8 | bytes[1];
        subIndex = bytes[3];
        switch (type) {
            case Type::RESP_WRITE_NORMAL: {
                data = 0;
                break;
            }
            case Type::RESP_READ_4BYTES: {
                data = bytes[7] << 24 | bytes[6] << 16 | bytes[5] << 8 | bytes[4];
                break;
            }
            case Type::RESP_READ_3BYTES: {
                data = bytes[6] << 16 | bytes[5] << 8 | bytes[4];
                break;
            }
            case Type::RESP_READ_2BYTES: {
                data = (uint32_t)bytes[5] << 8 | bytes[4];
                break;
            }
            case Type::RESP_READ_1BYTE: {
                data = bytes[4];
                break;
            }
            case Type::RESP_EXCEPTION: {
                data = bytes[7] << 24 | bytes[6] << 16 | bytes[5] << 8 | bytes[4];
                break;
            }
        }
    }

    // for request
    Message(uint32_t idCob, Type type, uint32_t index, uint32_t subIndex, uint32_t data = 0) :
        idCob(idCob), idCan(ToRequestCanId(idCob)), type(type), index(index), subIndex(subIndex), data(data) {}

    static uint32_t ToRequestCanId(uint32_t idCob) {
        return idCob + 0x600;
    }

    static uint32_t ToResponseCanId(uint32_t idCob) {
        return idCob + 0x580;
    }

    // for request
    virtual std::vector<uint8_t> ToBytes() {
        idCan = idCob + 0x600;
        std::vector<uint8_t> bytes;
        bytes.push_back((uint8_t)type);
        bytes.push_back(index & 0x00FF);
        bytes.push_back(index >> 8);
        bytes.push_back(subIndex);
        switch (type) {
            case Type::REQ_READ: {
                bytes.push_back(0x00);
                bytes.push_back(0x00);
                bytes.push_back(0x00);
                bytes.push_back(0x00);
                break;
            }
            case Type::REQ_WRITE_4BYTES: {
                bytes.push_back(data & 0x00FF);
                bytes.push_back(data >> 8 & 0x00FF);
                bytes.push_back(data >> 16 & 0x00FF);
                bytes.push_back(data >> 24);
                break;
            }
            case Type::REQ_WRITE_3BYTES: {
                bytes.push_back(data & 0x00FF);
                bytes.push_back(data >> 8 & 0x00FF);
                bytes.push_back(data >> 16 & 0x00FF);
                bytes.push_back(0x00);
                break;
            }
            case Type::REQ_WRITE_2BYTES: {
                bytes.push_back(data & 0x00FF);
                bytes.push_back(data >> 8 & 0x00FF);
                bytes.push_back(0x00);
                bytes.push_back(0x00);
                break;
            }
            case Type::REQ_WRITE_1BYTE: {
                bytes.push_back(data & 0x00FF);
                bytes.push_back(0x00);
                bytes.push_back(0x00);
                bytes.push_back(0x00);
                break;
            }
        }

        return bytes;
    }
};

}
}