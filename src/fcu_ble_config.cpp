#include "fcu_ble_config.h"

#include "fcu_configurator.h"

#if ENABLE_BLE_CONFIG
#include <BLEAdvertising.h>
#include <BLECharacteristic.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLEUtils.h>
#endif

namespace fcu_ble_config {

namespace {

constexpr const char* kDeviceName = "AeroForge FCU";
constexpr const char* kServiceUuid = "b7f3b8f0-6e6a-4d7a-9df7-2c13f0c00001";
constexpr const char* kRxUuid = "b7f3b8f1-6e6a-4d7a-9df7-2c13f0c00001";
constexpr const char* kTxUuid = "b7f3b8f2-6e6a-4d7a-9df7-2c13f0c00001";
constexpr const char* kInfoUuid = "b7f3b8f3-6e6a-4d7a-9df7-2c13f0c00001";
constexpr size_t kRxRingSize = 1024;
constexpr size_t kNotifyChunkBytes = 160;
constexpr uint16_t kFastBlinkOnMs = 62;
constexpr uint16_t kFastBlinkOffMs = 62;

Status gStatus;
bool gRequestedEnabled = (BLE_DEFAULT_ENABLED != 0);
bool gActive = false;
bool gConnected = false;
bool gRawButtonState = true;
bool gStableButtonState = true;
bool gLastReleasedState = true;
uint32_t gButtonChangeMs = 0;
uint8_t gClickCount = 0;
uint32_t gClickWindowStartMs = 0;
bool gLedOn = false;
uint32_t gLedPhaseStartMs = 0;

void writeLedPin(int pin, bool on) {
  if (pin < 0) return;
  digitalWrite(pin, (on != (FCU_BLE_LED_ACTIVE_LOW != 0)) ? HIGH : LOW);
}

void writeBleLeds(bool on) {
  writeLedPin(FCU_BLE_STATUS_LED_A, on);
  if (FCU_BLE_STATUS_LED_B != FCU_BLE_STATUS_LED_A) {
    writeLedPin(FCU_BLE_STATUS_LED_B, on);
  }
}

#if ENABLE_BLE_CONFIG
class BleConfigStream final : public Stream {
 public:
  void setTxCharacteristic(BLECharacteristic* tx) {
    tx_ = tx;
  }

  void setConnected(bool connected) {
    connected_ = connected;
    if (!connected_) {
      head_ = 0;
      tail_ = 0;
    }
  }

  int available() override {
    if (head_ >= tail_) return static_cast<int>(head_ - tail_);
    return static_cast<int>(sizeof(rx_) - tail_ + head_);
  }

  int read() override {
    if (tail_ == head_) return -1;
    const uint8_t b = rx_[tail_];
    tail_ = (tail_ + 1U) % sizeof(rx_);
    return b;
  }

  int peek() override {
    if (tail_ == head_) return -1;
    return rx_[tail_];
  }

  void flush() override {}

  size_t write(uint8_t b) override {
    return write(&b, 1);
  }

  size_t write(const uint8_t* buffer, size_t size) override {
    if (!connected_ || tx_ == nullptr || buffer == nullptr || size == 0U) {
      return 0;
    }
    size_t sent = 0;
    while (sent < size) {
      const size_t remaining = size - sent;
      const size_t chunk = remaining > kNotifyChunkBytes ? kNotifyChunkBytes : remaining;
      tx_->setValue(&buffer[sent], chunk);
      tx_->notify();
      sent += chunk;
      gStatus.txBytes += static_cast<uint32_t>(chunk);
    }
    return size;
  }

  int availableForWrite() override {
    return connected_ ? 512 : 0;
  }

  void pushRx(const uint8_t* data, size_t len) {
    if (data == nullptr) return;
    for (size_t i = 0; i < len; ++i) {
      const size_t next = (head_ + 1U) % sizeof(rx_);
      if (next == tail_) {
        gStatus.droppedRxBytes++;
        continue;
      }
      rx_[head_] = data[i];
      head_ = next;
      gStatus.rxBytes++;
    }
  }

 private:
  BLECharacteristic* tx_ = nullptr;
  bool connected_ = false;
  uint8_t rx_[kRxRingSize] = {};
  volatile size_t head_ = 0;
  volatile size_t tail_ = 0;
};

BleConfigStream gBleStream;
BLEServer* gServer = nullptr;
BLECharacteristic* gTx = nullptr;
BLECharacteristic* gRx = nullptr;
BLECharacteristic* gInfo = nullptr;

class ServerCallbacks final : public BLEServerCallbacks {
 public:
  void onConnect(BLEServer* server) override {
    (void)server;
    gConnected = true;
    gBleStream.setConnected(true);
  }

  void onDisconnect(BLEServer* server) override {
    (void)server;
    gConnected = false;
    gBleStream.setConnected(false);
    if (gActive) {
      BLEDevice::startAdvertising();
    }
  }
};

class RxCallbacks final : public BLECharacteristicCallbacks {
 public:
  void onWrite(BLECharacteristic* characteristic) override {
    if (characteristic == nullptr) return;
    gBleStream.pushRx(characteristic->getData(), characteristic->getLength());
  }
};

ServerCallbacks gServerCallbacks;
RxCallbacks gRxCallbacks;

bool startBle(uint32_t nowMs) {
  if (gActive) return true;
  if (!BLEDevice::init(kDeviceName)) {
    return false;
  }
  (void)BLEDevice::setMTU(185);
  gServer = BLEDevice::createServer();
  if (gServer == nullptr) {
    BLEDevice::deinit(false);
    return false;
  }
  gServer->setCallbacks(&gServerCallbacks);
  BLEService* service = gServer->createService(kServiceUuid);
  if (service == nullptr) {
    BLEDevice::deinit(false);
    return false;
  }
  gRx = service->createCharacteristic(kRxUuid,
                                      BLECharacteristic::PROPERTY_WRITE |
                                      BLECharacteristic::PROPERTY_WRITE_NR);
  gTx = service->createCharacteristic(kTxUuid, BLECharacteristic::PROPERTY_NOTIFY);
  gInfo = service->createCharacteristic(kInfoUuid, BLECharacteristic::PROPERTY_READ);
  if (gRx == nullptr || gTx == nullptr || gInfo == nullptr) {
    BLEDevice::deinit(false);
    return false;
  }
  gRx->setCallbacks(&gRxCallbacks);
  gInfo->setValue("AeroForge ESP32-S3 Mini FCU");
  gBleStream.setTxCharacteristic(gTx);
  service->start();
  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  if (advertising != nullptr) {
    advertising->addServiceUUID(kServiceUuid);
    advertising->setScanResponse(true);
  }
  BLEDevice::startAdvertising();
  gActive = true;
  gConnected = false;
  gLedPhaseStartMs = nowMs;
  return true;
}

void stopBle() {
  if (!gActive) return;
  BLEDevice::stopAdvertising();
  gBleStream.setConnected(false);
  gServer = nullptr;
  gTx = nullptr;
  gRx = nullptr;
  gInfo = nullptr;
  gConnected = false;
  gActive = false;
  BLEDevice::deinit(false);
  writeBleLeds(false);
}
#else
bool startBle(uint32_t) { return false; }
void stopBle() {}
#endif

void pollButton(uint32_t nowMs, bool allowToggle) {
  const bool rawReleased = digitalRead(FCU_BLE_BOOT_BUTTON_PIN) != LOW;
  if (rawReleased != gRawButtonState) {
    gRawButtonState = rawReleased;
    gButtonChangeMs = nowMs;
  }
  if ((nowMs - gButtonChangeMs) < BLE_CLICK_DEBOUNCE_MS) {
    return;
  }
  if (gStableButtonState == gRawButtonState) {
    return;
  }
  gStableButtonState = gRawButtonState;
  if (!gStableButtonState || gLastReleasedState == gStableButtonState) {
    gLastReleasedState = gStableButtonState;
    return;
  }
  gLastReleasedState = gStableButtonState;
  if (gClickCount == 0U || (nowMs - gClickWindowStartMs) > BLE_MULTI_CLICK_WINDOW_MS) {
    gClickWindowStartMs = nowMs;
    gClickCount = 0;
  }
  gClickCount++;
  if (gClickCount >= BLE_TOGGLE_CLICK_COUNT) {
    if (allowToggle) {
      gRequestedEnabled = !gRequestedEnabled;
      gStatus.toggles++;
    }
    gClickCount = 0;
  }
}

void serviceLeds(uint32_t nowMs) {
  if (!gActive) {
    if (gLedOn) {
      gLedOn = false;
      writeBleLeds(false);
    }
    return;
  }
  if (gConnected) {
    if (!gLedOn) {
      gLedOn = true;
      writeBleLeds(true);
    }
    return;
  }
  const uint16_t phaseMs = gLedOn ? kFastBlinkOnMs : kFastBlinkOffMs;
  if (gLedPhaseStartMs == 0U || (nowMs - gLedPhaseStartMs) >= phaseMs) {
    gLedPhaseStartMs = nowMs;
    gLedOn = !gLedOn;
    writeBleLeds(gLedOn);
  }
}

}  // namespace

void init(uint32_t nowMs) {
  gStatus.compiled = ENABLE_BLE_CONFIG != 0;
  gRequestedEnabled = (BLE_DEFAULT_ENABLED != 0);
  pinMode(FCU_BLE_BOOT_BUTTON_PIN, INPUT_PULLUP);
  if (FCU_BLE_STATUS_LED_A >= 0) pinMode(FCU_BLE_STATUS_LED_A, OUTPUT);
  if (FCU_BLE_STATUS_LED_B >= 0 && FCU_BLE_STATUS_LED_B != FCU_BLE_STATUS_LED_A) {
    pinMode(FCU_BLE_STATUS_LED_B, OUTPUT);
  }
  writeBleLeds(false);
  gButtonChangeMs = nowMs;
}

void service(uint32_t nowMs, bool allowToggle) {
#if ENABLE_BLE_CONFIG
  pollButton(nowMs, allowToggle);
  if (gRequestedEnabled && !gActive) {
    (void)startBle(nowMs);
  } else if (!gRequestedEnabled && gActive) {
    stopBle();
  }
  if (gActive) {
    fcu_configurator::service(gBleStream, nowMs, "ble", true);
  }
  serviceLeds(nowMs);
#else
  (void)nowMs;
  (void)allowToggle;
#endif
}

bool setRequestedEnabled(bool enabled) {
  gRequestedEnabled = enabled;
  return true;
}

bool requestedEnabled() {
  return gRequestedEnabled;
}

bool active() {
  return gActive;
}

bool connected() {
  return gConnected;
}

bool ledOverrideActive() {
  return gActive;
}

Status status() {
  gStatus.compiled = ENABLE_BLE_CONFIG != 0;
  gStatus.requestedEnabled = gRequestedEnabled;
  gStatus.active = gActive;
  gStatus.connected = gConnected;
  return gStatus;
}

}  // namespace fcu_ble_config
