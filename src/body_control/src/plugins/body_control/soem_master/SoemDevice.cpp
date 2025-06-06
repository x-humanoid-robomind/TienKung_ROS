#include "SoemDevice.h"

SoemDevice::SoemDevice() {
    ready = false;
}

bool SoemDevice::IsReady() {
    return ready;
}