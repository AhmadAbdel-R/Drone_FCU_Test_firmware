#include "fcu_ble_config.h"

#include "fcu_configurator.h"

#if ENABLE_BLE_CONFIG
// Direct ESP-IDF/NimBLE host APIs. We deliberately do NOT use the arduino-esp32
// BLE wrapper (BLEDevice/BLEServer/BLECharacteristic): that wrapper's NimBLE
// backend mismanaged the TX CCCD/subscribe path (Windows/Web Bluetooth saw
// "subscribed: false" forever and startNotifications() hung). Talking to the
// NimBLE host directly lets the stack auto-create exactly one CCCD for the TX
// characteristic and reports the real cur_notify/cur_indicate state, so
// subscriptions work reliably. NimBLE is the active BT host on this framework
// (arduino-esp32 3.3.x, CONFIG_NIMBLE_ENABLED=y); these headers are the same
// ones the wrapper itself pulls in.
#include <string.h>

// Pull in the same Arduino constructor the BLE wrapper uses so BTDM memory is
// kept reserved. NimBLE owns controller init; calling btStart() double-inits it.
#include "esp32-hal-bt-mem.h"
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/ble_hs_id.h>
#include <host/ble_hs_adv.h>
#include <host/ble_hs_mbuf.h>
#include <host/ble_uuid.h>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_att.h>
#include <host/util/util.h>
#include <os/os_mbuf.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#endif

namespace fcu_ble_config {

namespace {

constexpr const char* kDeviceName = "AeroForge FCU";
constexpr const char* kInfoValue = "AeroForge ESP32-S3 Mini FCU";
constexpr size_t kRxRingSize = 1024;
constexpr size_t kNotifyChunkBytes = 160;     // < MTU(185) - 3 ATT header bytes
constexpr uint16_t kPreferredMtu = 185;
constexpr uint16_t kAdvMinInterval = 0x20;    // units of 0.625 ms -> 20 ms
constexpr uint16_t kAdvMaxInterval = 0x40;    // 40 ms
constexpr uint16_t kFastBlinkOnMs = 62;
constexpr uint16_t kFastBlinkOffMs = 62;

Status gStatus;
bool gRequestedEnabled = (BLE_DEFAULT_ENABLED != 0);
PersistRequestedFn gPersistRequested = nullptr;
bool gActive = false;
volatile bool gConnected = false;
bool gInitialized = false;
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

void persistRequestedEnabled() {
  if (gPersistRequested != nullptr) {
    (void)gPersistRequested(gRequestedEnabled);
  }
}

#if ENABLE_BLE_CONFIG

// ---- 128-bit service/characteristic UUIDs --------------------------------
// NimBLE stores 128-bit UUIDs little-endian (LSB first), i.e. the reverse of
// the human-readable big-endian string. Only the time-low low byte differs
// between the four UUIDs (..f0/f1/f2/f3), which lands at value[12] below.
//   Service: b7f3b8f0-6e6a-4d7a-9df7-2c13f0c00001
//   RX:      b7f3b8f1-6e6a-4d7a-9df7-2c13f0c00001
//   TX:      b7f3b8f2-6e6a-4d7a-9df7-2c13f0c00001
//   INFO:    b7f3b8f3-6e6a-4d7a-9df7-2c13f0c00001
const ble_uuid128_t kServiceUuid = BLE_UUID128_INIT(
    0x01, 0x00, 0xc0, 0xf0, 0x13, 0x2c, 0xf7, 0x9d,
    0x7a, 0x4d, 0x6a, 0x6e, 0xf0, 0xb8, 0xf3, 0xb7);
const ble_uuid128_t kRxUuid = BLE_UUID128_INIT(
    0x01, 0x00, 0xc0, 0xf0, 0x13, 0x2c, 0xf7, 0x9d,
    0x7a, 0x4d, 0x6a, 0x6e, 0xf1, 0xb8, 0xf3, 0xb7);
const ble_uuid128_t kTxUuid = BLE_UUID128_INIT(
    0x01, 0x00, 0xc0, 0xf0, 0x13, 0x2c, 0xf7, 0x9d,
    0x7a, 0x4d, 0x6a, 0x6e, 0xf2, 0xb8, 0xf3, 0xb7);
const ble_uuid128_t kInfoUuid = BLE_UUID128_INIT(
    0x01, 0x00, 0xc0, 0xf0, 0x13, 0x2c, 0xf7, 0x9d,
    0x7a, 0x4d, 0x6a, 0x6e, 0xf3, 0xb8, 0xf3, 0xb7);

// Attribute value handles, filled in by NimBLE during ble_gatts_add_svcs /
// host start. The CCCD for TX is implicitly at gTxValHandle + 1.
uint16_t gRxValHandle = 0;
uint16_t gTxValHandle = 0;
uint16_t gInfoValHandle = 0;

// Connection / subscription state. Written from the NimBLE host task (GAP
// callback), read from the loop() task (TX writes / status). Plain volatile
// flags are sufficient for this single-central SPSC usage.
volatile uint16_t gConnHandle = BLE_HS_CONN_HANDLE_NONE;
volatile bool gTxSubscribed = false;   // client enabled notify or indicate
volatile bool gTxIndicate = false;     // client chose indicate over notify
volatile bool gSynced = false;         // host<->controller synced, addr ready
bool gHostInited = false;              // NimBLE host brought up once (sticky)
uint8_t gOwnAddrType = BLE_OWN_ADDR_PUBLIC;
volatile bool gAdvertiseRequested = false;
uint32_t gNextAdvAttemptMs = 0;

int gattAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                 struct ble_gatt_access_ctxt* ctxt, void* arg);
int gapEventCb(struct ble_gap_event* event, void* arg);
void requestAdvertising() {
  gAdvertiseRequested = true;
}

// GATT table. NimBLE adds the TX CCCD (0x2902) automatically because TX has
// the notify/indicate flags — we must NOT add one ourselves (that double-CCCD
// is exactly what broke the wrapper path).
const struct ble_gatt_chr_def kChrs[] = {
    {
        // RX: configurator -> FCU (write / write-without-response)
        .uuid = &kRxUuid.u,
        .access_cb = gattAccessCb,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .val_handle = &gRxValHandle,
    },
    {
        // TX: FCU -> configurator (notify, with indicate offered for Windows)
        .uuid = &kTxUuid.u,
        .access_cb = gattAccessCb,
        .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
        .val_handle = &gTxValHandle,
    },
    {
        // INFO: static identification string (read)
        .uuid = &kInfoUuid.u,
        .access_cb = gattAccessCb,
        .flags = BLE_GATT_CHR_F_READ,
        .val_handle = &gInfoValHandle,
    },
    {0},
};

const struct ble_gatt_svc_def kGattSvcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &kServiceUuid.u,
        .characteristics = kChrs,
    },
    {0},
};

class BleConfigStream final : public Stream {
 public:
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
    if (buffer == nullptr || size == 0U) {
      return 0;
    }
    if (!gConnected || gConnHandle == BLE_HS_CONN_HANDLE_NONE) {
      return 0;
    }
    if (!gTxSubscribed) {
      // Connected but the client has not enabled notifications/indications.
      // Never push a notification before the subscribe (requirement #5); drop
      // the reply and report it consumed so the caller does not stall retrying.
      gStatus.droppedTxBytes += static_cast<uint32_t>(size);
      return size;
    }
    size_t sent = 0;
    while (sent < size) {
      const size_t remaining = size - sent;
      const size_t chunk = remaining > kNotifyChunkBytes ? kNotifyChunkBytes : remaining;
      // ble_gatts_notify_custom / indicate_custom take ownership of the mbuf and
      // free it on every path, so we never os_mbuf_free it ourselves.
      struct os_mbuf* om = ble_hs_mbuf_from_flat(&buffer[sent], chunk);
      if (om == nullptr) {
        gStatus.droppedTxBytes += static_cast<uint32_t>(remaining);
        break;  // mbuf pool exhausted; caller may retry
      }
      // The shipping AeroForge Configurator (Web Bluetooth) calls
      // startNotifications(), which enables NOTIFY because TX advertises the
      // notify property -> gTxIndicate stays false and we take the notify path
      // (fire-and-forget, no per-packet ATT confirmation). The indicate branch
      // exists only for an indicate-only client; note that NimBLE allows just
      // ONE outstanding indication per connection, so a reply spanning multiple
      // chunks would drop chunks 2+ here (rc != 0 -> droppedTxBytes) until each
      // prior indication is confirmed. Acceptable as-is because no shipping
      // client subscribes via indicate; revisit with a NOTIFY_TX-driven TX
      // queue if an indicate-only client is ever added.
      const int rc = gTxIndicate
                         ? ble_gatts_indicate_custom(gConnHandle, gTxValHandle, om)
                         : ble_gatts_notify_custom(gConnHandle, gTxValHandle, om);
      if (rc != 0) {
        gStatus.droppedTxBytes += static_cast<uint32_t>(remaining);
        break;
      }
      sent += chunk;
      gStatus.txBytes += static_cast<uint32_t>(chunk);
    }
    return sent;
  }

  int availableForWrite() override {
    return gConnected ? 512 : 0;
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
  bool connected_ = false;
  uint8_t rx_[kRxRingSize] = {};
  volatile size_t head_ = 0;
  volatile size_t tail_ = 0;
};

BleConfigStream gBleStream;

// Single access callback for all three characteristics; dispatch on the value
// handle that NimBLE assigned at registration time.
int gattAccessCb(uint16_t conn_handle, uint16_t attr_handle,
                 struct ble_gatt_access_ctxt* ctxt, void* arg) {
  (void)conn_handle;
  (void)arg;
  switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      if (attr_handle == gRxValHandle) {
        uint8_t buf[256];  // bounded by MTU(185) -> max single write 182 bytes
        uint16_t outLen = 0;
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &outLen) == 0) {
          Serial.printf("[BLE] gatt write rx conn=%u len=%u\n",
                        static_cast<unsigned>(conn_handle),
                        static_cast<unsigned>(outLen));
          gBleStream.pushRx(buf, outLen);
        }
        return 0;
      }
      return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    case BLE_GATT_ACCESS_OP_READ_CHR:
      if (attr_handle == gInfoValHandle) {
        Serial.printf("[BLE] gatt read info conn=%u\n",
                      static_cast<unsigned>(conn_handle));
        const int rc = os_mbuf_append(ctxt->om, kInfoValue, strlen(kInfoValue));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
      }
      if (attr_handle == gTxValHandle) {
        Serial.printf("[BLE] gatt read tx-empty conn=%u\n",
                      static_cast<unsigned>(conn_handle));
        return 0;  // notify/indicate only; a direct read yields an empty value
      }
      return BLE_ATT_ERR_READ_NOT_PERMITTED;
    default:
      return BLE_ATT_ERR_UNLIKELY;
  }
}

bool startAdvertising() {
  if (!gSynced || !gActive || gConnected) return false;
  if (ble_gap_adv_active()) return true;

  // Main advertisement carries the flags + the 128-bit service UUID so
  // Web Bluetooth can filter on it. The full device name is too large to also
  // fit (16-byte UUID + name > 31 bytes), so the name goes in the scan response.
  struct ble_hs_adv_fields adv_fields;
  memset(&adv_fields, 0, sizeof(adv_fields));
  adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  adv_fields.uuids128 = &kServiceUuid;
  adv_fields.num_uuids128 = 1;
  adv_fields.uuids128_is_complete = 1;
  int rc = ble_gap_adv_set_fields(&adv_fields);
  if (rc != 0) {
    Serial.printf("[BLE] adv fields failed rc=%d\n", rc);
    return false;
  }

  struct ble_hs_adv_fields rsp_fields;
  memset(&rsp_fields, 0, sizeof(rsp_fields));
  rsp_fields.name = reinterpret_cast<const uint8_t*>(kDeviceName);
  rsp_fields.name_len = static_cast<uint8_t>(strlen(kDeviceName));
  rsp_fields.name_is_complete = 1;
  rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
  if (rc != 0) {
    Serial.printf("[BLE] scan response failed rc=%d\n", rc);
    return false;
  }

  struct ble_gap_adv_params adv_params;
  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   // connectable, undirected
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   // general discoverable
  adv_params.itvl_min = kAdvMinInterval;
  adv_params.itvl_max = kAdvMaxInterval;

  rc = ble_gap_adv_start(gOwnAddrType, nullptr, BLE_HS_FOREVER, &adv_params,
                         gapEventCb, nullptr);
  if (rc != 0) {
    Serial.printf("[BLE] adv start failed rc=%d synced=%u active=%u connected=%u\n",
                  rc,
                  static_cast<unsigned>(gSynced),
                  static_cast<unsigned>(gActive),
                  static_cast<unsigned>(gConnected));
    return false;
  }
  Serial.printf("[BLE] advertising started addr_type=%u\n",
                static_cast<unsigned>(gOwnAddrType));
  return true;
}

void serviceAdvertising(uint32_t nowMs) {
  if (!gAdvertiseRequested) return;
  if (!gSynced || !gActive || gConnected) return;
  if (ble_gap_adv_active()) {
    gAdvertiseRequested = false;
    return;
  }
  if (nowMs < gNextAdvAttemptMs) return;
  if (startAdvertising()) {
    gAdvertiseRequested = false;
  } else {
    gNextAdvAttemptMs = nowMs + 250U;
  }
}

int gapEventCb(struct ble_gap_event* event, void* arg) {
  (void)arg;
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        gConnHandle = event->connect.conn_handle;
        gTxSubscribed = false;
        gTxIndicate = false;
        gConnected = true;
        gBleStream.setConnected(true);
        Serial.printf("[BLE] connect ok handle=%u\n",
                      static_cast<unsigned>(gConnHandle));
      } else {
        // Connection failed to establish; resume advertising.
        gConnHandle = BLE_HS_CONN_HANDLE_NONE;
        gConnected = false;
        Serial.printf("[BLE] connect failed status=%d; advertising retry queued\n",
                      event->connect.status);
        requestAdvertising();
      }
      return 0;
    case BLE_GAP_EVENT_DISCONNECT:
      gConnected = false;
      gTxSubscribed = false;
      gTxIndicate = false;
      gConnHandle = BLE_HS_CONN_HANDLE_NONE;
      gStatus.negotiatedMtu = 0;
      gBleStream.setConnected(false);
      Serial.printf("[BLE] disconnect reason=%d; advertising retry queued\n",
                    event->disconnect.reason);
      requestAdvertising();
      return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
      // The subscribe event reports the characteristic VALUE handle (the CCCD
      // it backs is at value+1). cur_notify/cur_indicate are the live state.
      if (event->subscribe.attr_handle == gTxValHandle) {
        gTxIndicate = event->subscribe.cur_indicate != 0;
        gTxSubscribed = (event->subscribe.cur_notify != 0) ||
                        (event->subscribe.cur_indicate != 0);
        Serial.printf("[BLE] subscribe tx notify=%u indicate=%u subscribed=%u\n",
                      static_cast<unsigned>(event->subscribe.cur_notify != 0),
                      static_cast<unsigned>(event->subscribe.cur_indicate != 0),
                      static_cast<unsigned>(gTxSubscribed));
      } else {
        Serial.printf("[BLE] subscribe other attr=%u notify=%u indicate=%u\n",
                      static_cast<unsigned>(event->subscribe.attr_handle),
                      static_cast<unsigned>(event->subscribe.cur_notify != 0),
                      static_cast<unsigned>(event->subscribe.cur_indicate != 0));
      }
      return 0;
    case BLE_GAP_EVENT_MTU:
      gStatus.negotiatedMtu = event->mtu.value;
      Serial.printf("[BLE] mtu conn=%u value=%u\n",
                    static_cast<unsigned>(event->mtu.conn_handle),
                    static_cast<unsigned>(event->mtu.value));
      return 0;
    case BLE_GAP_EVENT_ADV_COMPLETE:
      Serial.printf("[BLE] advertising complete reason=%d; retry queued\n",
                    event->adv_complete.reason);
      requestAdvertising();
      return 0;
    default:
      return 0;
  }
}

void onSync() {
  // Ensure we have an identity address, then pick the address type to use.
  (void)ble_hs_util_ensure_addr(0);
  if (ble_hs_id_infer_auto(0, &gOwnAddrType) != 0) {
    gOwnAddrType = BLE_OWN_ADDR_PUBLIC;
  }
  gSynced = true;
  Serial.printf("[BLE] host synced addr_type=%u\n",
                static_cast<unsigned>(gOwnAddrType));
  requestAdvertising();
}

void onReset(int reason) {
  gSynced = false;
  Serial.printf("[BLE] host reset reason=%d\n", reason);
}

void hostTask(void* param) {
  (void)param;
  nimble_port_run();  // returns only after nimble_port_stop()
  nimble_port_freertos_deinit();
}

bool initHost() {
  Serial.printf("[BLE] init host start\n");
  const int initRc = nimble_port_init();
  if (initRc != 0) {
    Serial.printf("[BLE] nimble_port_init failed rc=%d\n", initRc);
    return false;
  }
  ble_hs_cfg.reset_cb = onReset;
  ble_hs_cfg.sync_cb = onSync;
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;  // "Just Works", no bonding
  ble_hs_cfg.sm_bonding = 0;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  int rc = ble_gatts_count_cfg(kGattSvcs);
  if (rc != 0) {
    Serial.printf("[BLE] gatts count failed rc=%d\n", rc);
    return false;
  }
  rc = ble_gatts_add_svcs(kGattSvcs);
  if (rc != 0) {
    Serial.printf("[BLE] gatts add failed rc=%d\n", rc);
    return false;
  }
  (void)ble_svc_gap_device_name_set(kDeviceName);
  (void)ble_att_set_preferred_mtu(kPreferredMtu);

  nimble_port_freertos_init(hostTask);
  Serial.printf("[BLE] init host done\n");
  return true;
}

bool startBle(uint32_t nowMs) {
  if (gActive) return true;
  if (!gHostInited) {
    if (!initHost()) {
      return false;
    }
    gHostInited = true;
  }
  gActive = true;
  gConnected = false;
  gLedPhaseStartMs = nowMs;
  // If the host is already synced (re-enable after a toggle-off), advertise now.
  // On the first enable, onSync() will start advertising once the stack is up.
  Serial.printf("[BLE] enabled\n");
  requestAdvertising();
  return true;
}

void stopBle() {
  if (!gActive) return;
  gActive = false;
  if (gConnected && gConnHandle != BLE_HS_CONN_HANDLE_NONE) {
    // 0x13 = Remote User Terminated Connection (HCI reason).
    (void)ble_gap_terminate(gConnHandle, 0x13);
  }
  if (ble_gap_adv_active()) {
    (void)ble_gap_adv_stop();
  }
  gConnected = false;
  gTxSubscribed = false;
  gTxIndicate = false;
  gConnHandle = BLE_HS_CONN_HANDLE_NONE;
  gStatus.negotiatedMtu = 0;
  gBleStream.setConnected(false);
  writeBleLeds(false);
  Serial.printf("[BLE] disabled\n");
}
#endif  // ENABLE_BLE_CONFIG

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
      persistRequestedEnabled();
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

void init(uint32_t nowMs, bool bootEnabled, PersistRequestedFn persistRequested) {
  gStatus.compiled = ENABLE_BLE_CONFIG != 0;
  gRequestedEnabled = bootEnabled;
  gPersistRequested = persistRequested;
  pinMode(FCU_BLE_BOOT_BUTTON_PIN, INPUT_PULLUP);
  if (FCU_BLE_STATUS_LED_A >= 0) pinMode(FCU_BLE_STATUS_LED_A, OUTPUT);
  if (FCU_BLE_STATUS_LED_B >= 0 && FCU_BLE_STATUS_LED_B != FCU_BLE_STATUS_LED_A) {
    pinMode(FCU_BLE_STATUS_LED_B, OUTPUT);
  }
  writeBleLeds(false);
  gRawButtonState = digitalRead(FCU_BLE_BOOT_BUTTON_PIN) != LOW;
  gStableButtonState = gRawButtonState;
  gLastReleasedState = gStableButtonState;
  gButtonChangeMs = nowMs;
  gClickCount = 0;
  gClickWindowStartMs = 0;
  gInitialized = true;
}

void service(uint32_t nowMs, bool allowToggle) {
#if ENABLE_BLE_CONFIG
  if (!gInitialized) return;
  pollButton(nowMs, allowToggle);
  if (gRequestedEnabled && !gActive) {
    (void)startBle(nowMs);
  } else if (!gRequestedEnabled && gActive) {
    stopBle();
  }
  serviceAdvertising(nowMs);
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
  if (gRequestedEnabled == enabled) return true;
  gRequestedEnabled = enabled;
  persistRequestedEnabled();
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
#if ENABLE_BLE_CONFIG
  gStatus.subscribed = gTxSubscribed;
#endif
  return gStatus;
}

}  // namespace fcu_ble_config
