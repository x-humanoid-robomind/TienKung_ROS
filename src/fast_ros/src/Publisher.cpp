#include "Publisher.h"

namespace fast_ros {

Publisher::Publisher() {
    funcPubFast = [](const std::any&){};
}

}