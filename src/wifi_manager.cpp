#include "wifi_manager.h"

#include "log_router.h"

#if FCU_WIFI_STACK_ENABLED

#include <Arduino.h>
#include <WiFi.h>

#include <lwip/sockets.h>

#include <atomic>
#include <cstring>

#if FCU_ENABLE_WIFI_OTA
#include <Update.h>
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#endif

namespace wifi_mgr {

namespace {

enum class State : uint8_t { Off = 0, Connecting, Up, Teardown };

Callbacks gCb;
const char* gSsid = "";
const char* gPass = "";
const char* gToken = "";

State gState = State::Off;
uint32_t gConnectDeadlineMs = 0;
uint32_t gRetryBackoffUntilMs = 0;

// Cross-task inputs. Producers: flight task (publishSafety) and CRSF task
// (requestEnable/requestDisable). Consumer: service() on loop().
std::atomic<bool> gWifiAllowed{false};
std::atomic<bool> gArmed{false};
std::atomic<bool> gRequestedOn{false};
// Set when an armed-forced teardown happens with the switch still ON: the
// switch must produce a fresh OFF edge (requestDisable) before a new ON is
// honored. Implements "WiFi never auto-resurrects after a disarm".
std::atomic<bool> gRequireFreshEdge{false};
std::atomic<bool> gRebootPending{false};
uint32_t gRebootAtMs = 0;

// UDP log sink.
int gUdpSock = -1;
sockaddr_in gUdpDest = {};

// Stats — written from service() (loop task) and the OTA handler (httpd
// task), so the counters are atomics.
std::atomic<uint32_t> gConnectAttempts{0};
std::atomic<uint32_t> gConnectFailures{0};
std::atomic<uint32_t> gForcedTeardownsArmed{0};
std::atomic<uint32_t> gEnableIgnoredArmed{0};
std::atomic<uint32_t> gEnableIgnoredEdge{0};
std::atomic<uint32_t> gUdpSendFailures{0};
std::atomic<uint32_t> gOtaAttempts{0};
std::atomic<uint32_t> gOtaRejected{0};

uint32_t gLastIgnoreLogMs = 0;

#if FCU_ENABLE_WIFI_OTA
httpd_handle_t gOtaServer = nullptr;

// Constant-time compare (same as pid_webserver) so the OTA token can't be
// guessed via response timing.
bool constantTimeEquals(const char* a, const char* b) {
  const size_t la = strlen(a);
  const size_t lb = strlen(b);
  uint8_t diff = static_cast<uint8_t>(la ^ lb);
  const size_t n = (la > lb) ? la : lb;
  for (size_t i = 0; i < n; ++i) {
    const uint8_t ca = (i < la) ? static_cast<uint8_t>(a[i]) : 0;
    const uint8_t cb = (i < lb) ? static_cast<uint8_t>(b[i]) : 0;
    diff |= static_cast<uint8_t>(ca ^ cb);
  }
  return diff == 0;
}

bool otaAuthorized(httpd_req_t* req) {
  if (gToken == nullptr || gToken[0] == '\0') {
    return false;  // fail closed: no token configured => no OTA
  }
  char hdr[96] = {};
  if (httpd_req_get_hdr_value_str(req, "X-Auth-Token", hdr, sizeof(hdr)) != ESP_OK) {
    return false;
  }
  return constantTimeEquals(hdr, gToken);
}

esp_err_t otaSendJson(httpd_req_t* req, const char* status, const char* body) {
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, body);
}

// True only in the exact state OTA is allowed to run: WiFi was brought up
// through the disarmed-only path AND the craft is still disarmed-safe now.
bool otaSafeNow() {
  return gWifiAllowed.load(std::memory_order_relaxed) &&
         !gArmed.load(std::memory_order_relaxed);
}

// GET /ota/status — read-only, unauthenticated (no secrets in the response).
esp_err_t handleOtaStatus(httpd_req_t* req) {
  const esp_partition_t* running = esp_ota_get_running_partition();
  char body[224];
  snprintf(body, sizeof(body),
           "{\"app\":\"fcu\",\"built\":\"%s %s\",\"partition\":\"%s\","
           "\"otaSafeNow\":%s,\"otaAttempts\":%lu}",
           __DATE__, __TIME__,
           (running != nullptr) ? running->label : "?",
           otaSafeNow() ? "true" : "false",
           static_cast<unsigned long>(gOtaAttempts.load()));
  return otaSendJson(req, "200 OK", body);
}

// POST /ota — streaming firmware upload.
//   curl -X POST --data-binary @firmware.bin \
//        -H "X-Auth-Token: <FCU_PID_AUTH_TOKEN>" http://<fcu-ip>/ota
// Gates (each checked again mid-transfer where it matters):
//   1. valid token   2. disarmed-safe   3. motors force-stopped
// Arming mid-upload aborts the write; the boot partition is only switched by
// a fully received, validated image (Update.end), after which the FCU reboots
// into the new firmware ~700 ms after the HTTP response.
esp_err_t handleOtaUpload(httpd_req_t* req) {
  gOtaAttempts.fetch_add(1, std::memory_order_relaxed);
  if (!otaAuthorized(req)) {
    gOtaRejected.fetch_add(1, std::memory_order_relaxed);
    return otaSendJson(req, "401 Unauthorized", "{\"error\":\"unauthorized\"}");
  }
  if (!otaSafeNow()) {
    gOtaRejected.fetch_add(1, std::memory_order_relaxed);
    return otaSendJson(req, "409 Conflict", "{\"error\":\"not_disarmed_safe\"}");
  }
  const size_t total = req->content_len;
  if (total == 0) {
    gOtaRejected.fetch_add(1, std::memory_order_relaxed);
    return otaSendJson(req, "400 Bad Request", "{\"error\":\"empty_body\"}");
  }

  // Belt-and-braces: motors are already stopped (disarmed gate), but flash
  // writes suspend the cache and stall other tasks ms-at-a-time, so make the
  // stop explicit and auditable before the first write.
  if (gCb.stopMotors != nullptr) {
    gCb.stopMotors("ota_upload");
  }
  fcu_log::logf(fcu_log::Level::Warn, "[OTA] upload start size=%u\n",
                static_cast<unsigned>(total));

  if (!Update.begin(total)) {  // validates size against the next OTA slot
    gOtaRejected.fetch_add(1, std::memory_order_relaxed);
    fcu_log::logf(fcu_log::Level::Error, "[OTA] begin failed: %s\n",
                  Update.errorString());
    return otaSendJson(req, "500 Internal Server Error", "{\"error\":\"begin_failed\"}");
  }

  static uint8_t chunk[4096];  // httpd is single-worker; static keeps its stack small
  size_t received = 0;
  while (received < total) {
    // Abort immediately if the craft armed mid-transfer. The partially
    // written slot is abandoned; the boot partition is untouched.
    if (!otaSafeNow()) {
      Update.abort();
      gOtaRejected.fetch_add(1, std::memory_order_relaxed);
      fcu_log::logf(fcu_log::Level::Warn, "[OTA] ABORTED: armed during upload (%u/%u bytes)\n",
                    static_cast<unsigned>(received), static_cast<unsigned>(total));
      return otaSendJson(req, "409 Conflict", "{\"error\":\"armed_during_upload\"}");
    }
    const size_t want = total - received;
    const int n = httpd_req_recv(req, reinterpret_cast<char*>(chunk),
                                 (want > sizeof(chunk)) ? sizeof(chunk) : want);
    if (n == HTTPD_SOCK_ERR_TIMEOUT) {
      continue;
    }
    if (n <= 0) {
      Update.abort();
      fcu_log::logf(fcu_log::Level::Error, "[OTA] recv failed at %u/%u bytes\n",
                    static_cast<unsigned>(received), static_cast<unsigned>(total));
      return otaSendJson(req, "400 Bad Request", "{\"error\":\"recv_failed\"}");
    }
    if (Update.write(chunk, static_cast<size_t>(n)) != static_cast<size_t>(n)) {
      Update.abort();
      fcu_log::logf(fcu_log::Level::Error, "[OTA] flash write failed: %s\n",
                    Update.errorString());
      return otaSendJson(req, "500 Internal Server Error", "{\"error\":\"write_failed\"}");
    }
    received += static_cast<size_t>(n);
  }

  if (!Update.end(true) || !Update.isFinished()) {
    fcu_log::logf(fcu_log::Level::Error, "[OTA] finalize failed: %s\n",
                  Update.errorString());
    return otaSendJson(req, "500 Internal Server Error", "{\"error\":\"finalize_failed\"}");
  }

  fcu_log::logf(fcu_log::Level::Warn,
                "[OTA] upload OK (%u bytes) — rebooting into new firmware\n",
                static_cast<unsigned>(received));
  const esp_err_t res = otaSendJson(req, "200 OK", "{\"ok\":true,\"rebooting\":true}");
  gRebootAtMs = millis() + 700U;  // let TCP flush + one last log drain
  gRebootPending.store(true, std::memory_order_relaxed);
  return res;
}

void startOtaServer() {
  if (gOtaServer != nullptr) {
    return;
  }
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = FCU_WIFI_OTA_HTTP_PORT;
  cfg.task_priority = 3;  // below flight(24)/radio(23)/sensor(8)/crsf(7)
  cfg.core_id = 0;        // keep off core 1 (flight + radio)
  cfg.stack_size = 8 * 1024;
  cfg.max_uri_handlers = 4;
  cfg.lru_purge_enable = true;
  if (httpd_start(&gOtaServer, &cfg) != ESP_OK) {
    gOtaServer = nullptr;
    fcu_log::logf(fcu_log::Level::Error, "[OTA] httpd_start failed\n");
    return;
  }
  httpd_uri_t status = {};
  status.uri = "/ota/status";
  status.method = HTTP_GET;
  status.handler = handleOtaStatus;
  httpd_register_uri_handler(gOtaServer, &status);
  httpd_uri_t upload = {};
  upload.uri = "/ota";
  upload.method = HTTP_POST;
  upload.handler = handleOtaUpload;
  httpd_register_uri_handler(gOtaServer, &upload);
}

void stopOtaServer() {
  if (gOtaServer != nullptr) {
    httpd_stop(gOtaServer);
    gOtaServer = nullptr;
  }
}
#endif  // FCU_ENABLE_WIFI_OTA

void openUdpLogSink() {
#if FCU_ENABLE_WIFI_LOGGING
  if (gUdpSock >= 0) {
    return;
  }
  gUdpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (gUdpSock < 0) {
    fcu_log::logf(fcu_log::Level::Error, "[WIFI] UDP socket create failed\n");
    return;
  }
  // Non-blocking: a congested stack drops datagrams instead of stalling loop().
  const int fl = fcntl(gUdpSock, F_GETFL, 0);
  fcntl(gUdpSock, F_SETFL, fl | O_NONBLOCK);

  memset(&gUdpDest, 0, sizeof(gUdpDest));
  gUdpDest.sin_family = AF_INET;
  gUdpDest.sin_port = htons(FCU_WIFI_LOG_UDP_PORT);
  const char* target = FCU_WIFI_LOG_UDP_TARGET;
  if (target[0] != '\0') {
    gUdpDest.sin_addr.s_addr = inet_addr(target);
  } else {
    // Subnet broadcast derived from the DHCP lease (e.g. 192.168.1.255).
    const uint32_t ip = static_cast<uint32_t>(WiFi.localIP());
    const uint32_t mask = static_cast<uint32_t>(WiFi.subnetMask());
    gUdpDest.sin_addr.s_addr = (ip & mask) | ~mask;
    const int one = 1;
    setsockopt(gUdpSock, SOL_SOCKET, SO_BROADCAST, &one, sizeof(one));
  }
  fcu_log::setWifiSinkActive(true);
  fcu_log::logf(fcu_log::Level::Warn,
                "[WIFI] UDP log sink up -> %s:%u (USB logs %s)\n",
                (target[0] != '\0') ? target : "subnet-broadcast",
                static_cast<unsigned>(FCU_WIFI_LOG_UDP_PORT),
                (FCU_USB_LOG_SUSPEND_WHEN_WIFI != 0) ? "reduced to Error/Critical"
                                                     : "unchanged");
#endif
}

void closeUdpLogSink() {
  fcu_log::setWifiSinkActive(false);
  if (gUdpSock >= 0) {
    close(gUdpSock);
    gUdpSock = -1;
  }
}

void teardown(const char* reason) {
  closeUdpLogSink();
#if FCU_ENABLE_WIFI_OTA
  stopOtaServer();
#endif
  WiFi.disconnect(true /* also power down WiFi */);
  WiFi.mode(WIFI_OFF);
  gState = State::Off;
  fcu_log::logf(fcu_log::Level::Warn, "[WIFI] off (%s)\n", reason);
}

const char* kStateNames[] = {"OFF", "CONNECTING", "UP", "TEARDOWN"};

}  // namespace

void init(const Callbacks& cbs, const char* ssid, const char* password,
          const char* authToken) {
  gCb = cbs;
  gSsid = (ssid != nullptr) ? ssid : "";
  gPass = (password != nullptr) ? password : "";
  gToken = (authToken != nullptr) ? authToken : "";
}

void publishSafety(bool wifiAllowed, bool armed) {
  gWifiAllowed.store(wifiAllowed, std::memory_order_relaxed);
  gArmed.store(armed, std::memory_order_relaxed);
}

void requestEnable(uint32_t nowMs) {
  if (gArmed.load(std::memory_order_relaxed)) {
    gEnableIgnoredArmed.fetch_add(1, std::memory_order_relaxed);
    if (nowMs - gLastIgnoreLogMs >= 5000U) {
      gLastIgnoreLogMs = nowMs;
      fcu_log::logf(fcu_log::Level::Warn,
                    "[WIFI] enable request IGNORED: craft is armed\n");
    }
    return;
  }
  if (gRequireFreshEdge.load(std::memory_order_relaxed)) {
    gEnableIgnoredEdge.fetch_add(1, std::memory_order_relaxed);
    if (nowMs - gLastIgnoreLogMs >= 5000U) {
      gLastIgnoreLogMs = nowMs;
      fcu_log::logf(fcu_log::Level::Warn,
                    "[WIFI] enable request ignored: flip the AUX switch OFF then ON "
                    "(fresh edge required after arming)\n");
    }
    return;
  }
  gRequestedOn.store(true, std::memory_order_relaxed);
}

void requestDisable() {
  gRequestedOn.store(false, std::memory_order_relaxed);
  gRequireFreshEdge.store(false, std::memory_order_relaxed);  // OFF edge seen
}

void service(uint32_t nowMs) {
  // Deferred reboot after a successful OTA (gives the HTTP response + final
  // log drain time to leave the box).
  if (gRebootPending.load(std::memory_order_relaxed) &&
      static_cast<int32_t>(nowMs - gRebootAtMs) >= 0) {
    ESP.restart();
  }

  const bool armed = gArmed.load(std::memory_order_relaxed);
  const bool allowed = gWifiAllowed.load(std::memory_order_relaxed);
  const bool wanted = gRequestedOn.load(std::memory_order_relaxed);

  // ---- Arming kills WiFi in ANY active state, immediately. -----------------
  if (armed && gState != State::Off) {
    gForcedTeardownsArmed.fetch_add(1, std::memory_order_relaxed);
    // The switch is presumably still ON: demand a fresh OFF->ON edge before
    // WiFi may return, so landing does not silently re-enable it.
    if (wanted) {
      gRequireFreshEdge.store(true, std::memory_order_relaxed);
      gRequestedOn.store(false, std::memory_order_relaxed);
    }
    teardown("armed");
    return;
  }

  switch (gState) {
    case State::Off:
      if (wanted && allowed && !armed &&
          static_cast<int32_t>(nowMs - gRetryBackoffUntilMs) >= 0) {
        if (gSsid[0] == '\0') {
          fcu_log::logf(fcu_log::Level::Error,
                        "[WIFI] enable requested but SSID is empty; check pidweb_secrets.h\n");
          gRequestedOn.store(false, std::memory_order_relaxed);
          return;
        }
        gConnectAttempts.fetch_add(1, std::memory_order_relaxed);
        WiFi.persistent(false);
        WiFi.mode(WIFI_STA);
        WiFi.begin(gSsid, gPass);  // asynchronous — status polled below
        gConnectDeadlineMs = nowMs + FCU_WIFI_CONNECT_TIMEOUT_MS;
        gState = State::Connecting;
        fcu_log::logf(fcu_log::Level::Warn, "[WIFI] connecting to '%s' (disarmed)\n", gSsid);
      }
      break;

    case State::Connecting:
      if (!wanted || !allowed) {
        teardown(!wanted ? "switch_off" : "not_allowed");
        break;
      }
      if (WiFi.status() == WL_CONNECTED) {
        gState = State::Up;
        fcu_log::logf(fcu_log::Level::Warn, "[WIFI] up ip=%s rssi=%ddBm\n",
                      WiFi.localIP().toString().c_str(), static_cast<int>(WiFi.RSSI()));
        openUdpLogSink();
#if FCU_ENABLE_WIFI_OTA
        startOtaServer();
        fcu_log::logf(fcu_log::Level::Warn, "[OTA] endpoint ready: POST http://%s:%u/ota\n",
                      WiFi.localIP().toString().c_str(),
                      static_cast<unsigned>(FCU_WIFI_OTA_HTTP_PORT));
#endif
      } else if (static_cast<int32_t>(nowMs - gConnectDeadlineMs) >= 0) {
        gConnectFailures.fetch_add(1, std::memory_order_relaxed);
        gRetryBackoffUntilMs = nowMs + FCU_WIFI_RETRY_BACKOFF_MS;
        teardown("connect_timeout");  // request stays ON -> retry after backoff
      }
      break;

    case State::Up:
      if (!wanted || !allowed) {
        teardown(!wanted ? "switch_off" : "not_allowed");
        break;
      }
      if (WiFi.status() != WL_CONNECTED) {
        gConnectFailures.fetch_add(1, std::memory_order_relaxed);
        gRetryBackoffUntilMs = nowMs + FCU_WIFI_RETRY_BACKOFF_MS;
        teardown("link_dropped");  // request stays ON -> retry after backoff
      }
      break;

    case State::Teardown:  // transient; teardown() returns straight to Off
      gState = State::Off;
      break;
  }
}

bool wifiUp() { return gState == State::Up; }

bool logSinkReady() { return gUdpSock >= 0; }

const char* stateName() {
  return kStateNames[static_cast<uint8_t>(gState)];
}

Stats stats() {
  Stats s;
  s.connectAttempts = gConnectAttempts.load(std::memory_order_relaxed);
  s.connectFailures = gConnectFailures.load(std::memory_order_relaxed);
  s.forcedTeardownsArmed = gForcedTeardownsArmed.load(std::memory_order_relaxed);
  s.enableIgnoredArmed = gEnableIgnoredArmed.load(std::memory_order_relaxed);
  s.enableIgnoredEdge = gEnableIgnoredEdge.load(std::memory_order_relaxed);
  s.udpSendFailures = gUdpSendFailures.load(std::memory_order_relaxed);
  s.otaAttempts = gOtaAttempts.load(std::memory_order_relaxed);
  s.otaRejected = gOtaRejected.load(std::memory_order_relaxed);
  return s;
}

void sendLogDatagram(const uint8_t* data, size_t len) {
  if (gUdpSock < 0 || data == nullptr || len == 0) {
    return;
  }
  // Non-blocking socket: EWOULDBLOCK / ERR_MEM under pressure just drops the
  // datagram. NEVER log from here (drain() -> sendLogDatagram recursion).
  const ssize_t sent = sendto(gUdpSock, data, len, 0,
                              reinterpret_cast<const sockaddr*>(&gUdpDest),
                              sizeof(gUdpDest));
  if (sent < 0 || static_cast<size_t>(sent) != len) {
    gUdpSendFailures.fetch_add(1, std::memory_order_relaxed);
  }
}

}  // namespace wifi_mgr

#else  // !FCU_WIFI_STACK_ENABLED

// Stubs so call sites compile unconditionally; the linker drops what is
// unreferenced. Mirrors the pid_webserver stub pattern.
namespace wifi_mgr {
void init(const Callbacks&, const char*, const char*, const char*) {}
void publishSafety(bool, bool) {}
void requestEnable(uint32_t) {}
void requestDisable() {}
void service(uint32_t) {}
bool wifiUp() { return false; }
bool logSinkReady() { return false; }
const char* stateName() { return "DISABLED"; }
Stats stats() { return Stats{}; }
void sendLogDatagram(const uint8_t*, size_t) {}
}  // namespace wifi_mgr

#endif  // FCU_WIFI_STACK_ENABLED
