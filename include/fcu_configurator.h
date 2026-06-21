#pragma once

#include <Arduino.h>

#include "pid_webserver.h"

#ifndef ENABLE_USB_CONFIG
#define ENABLE_USB_CONFIG 0
#endif
#ifndef ENABLE_BLE_CONFIG
#define ENABLE_BLE_CONFIG 0
#endif

#ifndef FCU_CONFIGURATOR_PROTOCOL_VERSION
#define FCU_CONFIGURATOR_PROTOCOL_VERSION 1
#endif

namespace fcu_configurator {

struct MotorTestCallbacks {
  bool (*arm)() = nullptr;
  bool (*set)(uint8_t motorMask, uint16_t raw, uint16_t timeoutMs) = nullptr;
  bool (*stop)() = nullptr;
};

struct SystemCallbacks {
  bool (*safeToMutate)() = nullptr;
  bool (*rescanSensors)() = nullptr;
  bool (*reboot)() = nullptr;
  bool (*enterBootloader)() = nullptr;
};

void init(const pid_webserver::Callbacks& callbacks, const char* buildName);
void setMotorTestCallbacks(const MotorTestCallbacks& callbacks);
void setSystemCallbacks(const SystemCallbacks& callbacks);
void service(Stream& serial, uint32_t nowMs, const char* transportName = "usb",
             bool bleTransport = false);
bool enabled();

}  // namespace fcu_configurator
