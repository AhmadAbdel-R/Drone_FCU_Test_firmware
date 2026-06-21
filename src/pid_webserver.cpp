#include "pid_webserver.h"

namespace pid_webserver {

void registerCallbacks(const Callbacks&) {}

void setAuthToken(const char*) {}

bool start(const char*, const char*, uint32_t) {
  return false;
}

void service(uint32_t) {}

void publishSafety(bool, bool) {}

bool running() {
  return false;
}

}  // namespace pid_webserver
