#pragma once

#include <Arduino.h>
#include <stdint.h>

#ifndef ENABLE_BLE_CONFIG
#define ENABLE_BLE_CONFIG 0
#endif

#ifndef BLE_TOGGLE_CLICK_COUNT
#define BLE_TOGGLE_CLICK_COUNT 3
#endif
#ifndef BLE_CLICK_DEBOUNCE_MS
#define BLE_CLICK_DEBOUNCE_MS 40U
#endif
#ifndef BLE_MULTI_CLICK_WINDOW_MS
#define BLE_MULTI_CLICK_WINDOW_MS 1200U
#endif
#ifndef BLE_DEFAULT_ENABLED
#define BLE_DEFAULT_ENABLED 0
#endif
#ifndef FCU_BLE_BOOT_BUTTON_PIN
#define FCU_BLE_BOOT_BUTTON_PIN 0
#endif
#ifndef FCU_BLE_STATUS_LED_A
#define FCU_BLE_STATUS_LED_A 47
#endif
#ifndef FCU_BLE_STATUS_LED_B
#define FCU_BLE_STATUS_LED_B 48
#endif
#ifndef FCU_BLE_LED_ACTIVE_LOW
#define FCU_BLE_LED_ACTIVE_LOW 0
#endif

namespace fcu_ble_config {

using PersistRequestedFn = bool (*)(bool enabled);

struct Status {
  bool compiled = false;
  bool requestedEnabled = false;
  bool active = false;
  bool connected = false;
  uint32_t rxBytes = 0;
  uint32_t txBytes = 0;
  uint32_t droppedRxBytes = 0;
  uint32_t toggles = 0;
};

void init(uint32_t nowMs, bool bootEnabled = (BLE_DEFAULT_ENABLED != 0),
          PersistRequestedFn persistRequested = nullptr);
void service(uint32_t nowMs, bool allowToggle);
bool setRequestedEnabled(bool enabled);
bool requestedEnabled();
bool active();
bool connected();
bool ledOverrideActive();
Status status();

}  // namespace fcu_ble_config
