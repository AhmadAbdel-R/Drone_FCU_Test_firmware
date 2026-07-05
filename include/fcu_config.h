#pragma once

// =============================================================================
// fcu_config — versioned, table-driven configuration registry.
//
// INAV-INSPIRED, CLEAN-ROOM. This reproduces the *behavior* of INAV's settings
// system (a flat registry of named, range-validated parameters with defaults,
// persistence, and full-dump export/import) without using any INAV source.
//
// DESIGN
//   * Every tunable is one ParamDef row: stable JSON name, stable NVS key,
//     min/max/default in a float value domain (bools are 0/1, enums are
//     integers). The table lives in flash; live values live in one RAM array.
//   * STORED params persist in this module's own NVS namespace ("fcucfg1").
//   * EXTERNAL params bind to values that already have an owner + NVS record
//     elsewhere (the 12 PID gains, notch config, motor trims, ...). The
//     registry reads/writes them through callbacks registered by main.cpp so
//     there is exactly ONE source of truth per value, but export/import still
//     covers the whole craft configuration in a single JSON document.
//   * config_version is persisted alongside the stored params. Imports carry
//     the version they were exported with; migrateImport() maps old names /
//     semantics forward before validation. Bumping kConfigVersion requires
//     adding a migration case there.
//
// SAFETY MODEL
//   * set/import REJECT out-of-range or non-finite values — never clamp
//     silently. An import is all-or-nothing: every pair is validated against
//     the table BEFORE anything is applied (rollback semantics).
//   * Before an import mutates anything, the current values are snapshotted;
//     any NVS write failure rolls RAM back to the snapshot.
//   * writeInProgress() is exposed as an arming gate: the flight code refuses
//     to arm while a config apply/import is mid-flight.
//   * Applying a param NEVER touches flight state directly. The registry
//     calls the registered group hook; main.cpp owns re-staging runtime state
//     (atomics / double-buffered configs) exactly like the dynamic-notch
//     pending-config pattern.
//
// THREADING
//   * All public functions are safe to call from the webserver / loop()
//     context. They take an internal FreeRTOS mutex.
//   * NOT safe to call from the flight task (may block on NVS). The flight
//     task consumes values via the staged runtime state, never this API.
// =============================================================================

#include <stddef.h>
#include <stdint.h>

namespace fcu_config {

// Bump when a param changes name/meaning/units. Add a migration in
// migrateImport() for every bump so old exported backups keep importing.
constexpr uint16_t kConfigVersion = 1;

// ---------------------------------------------------------------------------
// Param groups — the granularity of the "something changed" hook. main.cpp
// registers one callback and re-stages only the touched group's runtime.
// ---------------------------------------------------------------------------
enum Group : uint8_t {
  GROUP_SYSTEM = 0,   // board alignment
  GROUP_MOTOR,        // protocol/order/direction/idle/test values
  GROUP_MIXER,        // 4x4 mixer table
  GROUP_FILTER,       // gyro/accel/dterm/setpoint filter chain
  GROUP_MODES,        // aux-range mode slots
  GROUP_GPS,          // provider/baud/rate/model/sbas
  GROUP_ARMING,       // arming safety gates
  GROUP_NAV,          // POSHOLD/RTH limits + quality gates
  GROUP_ALT,          // altitude source / rangefinder
  GROUP_MAG,          // mag alignment + cal window
  GROUP_BLACKBOX,     // logging
  GROUP_PID_EXT,      // external: rate/angle PID gains
  GROUP_TUNE_EXT,     // external: mixer bias/trims/mag/notch/failsafe
  GROUP_NAVPID_EXT,   // external: velocity/position/landing gains
  GROUP_COUNT,
};

// ---------------------------------------------------------------------------
// External-binding ids. Rows with `ext != EXT_NONE` route get/set through the
// ExternalBindings dispatcher instead of the stored-value array. Stable ids —
// append only.
// ---------------------------------------------------------------------------
enum ExtId : uint16_t {
  EXT_NONE = 0xFFFF,
  // Rate/angle PID gains (milli-units, matches fcu_nvs field indices 0..11).
  EXT_PID_BASE = 0,          // EXT_PID_BASE + fieldIdx, 12 slots
  // Mixer / trims / misc tune values.
  EXT_MIX_PITCH_FRONT_BIAS = 20,
  EXT_MOTOR_TRIM_1 = 21,     // ..24 = trims 1..4
  EXT_MAG_YAW_GAIN = 25,
  EXT_MAG_HEADING_TRIM = 26,
  EXT_FAILSAFE_BYPASS = 27,
  EXT_NOTCH_ENABLE = 28,
  EXT_NOTCH_MIN_HZ = 29,
  EXT_NOTCH_MAX_HZ = 30,
  EXT_NOTCH_Q = 31,
  // Autonomy controller gains (fcu_nvs Velocity/Position/Landing records).
  EXT_VEL_H_P = 40, EXT_VEL_H_I = 41, EXT_VEL_H_D = 42,
  EXT_VEL_V_P = 43, EXT_VEL_V_I = 44, EXT_VEL_V_D = 45,
  EXT_POS_P = 46,   EXT_POS_I = 47,   EXT_POS_D = 48,
  EXT_LAND_P = 49,  EXT_LAND_I = 50,  EXT_LAND_D = 51, EXT_LAND_HOVER = 52,
};

struct ParamDef {
  const char* name;    // stable JSON name (snake_case)
  const char* nvsKey;  // stable NVS key (<=15 chars); nullptr for EXTERNAL
  float min;
  float max;
  float def;
  uint8_t decimals;    // JSON formatting: 0 => integer
  uint8_t group;       // Group enum
  uint16_t ext;        // ExtId or EXT_NONE
};

// External-value dispatcher, registered by main.cpp. get/set operate on the
// live RAM value (set == "apply"); save persists every external record whose
// group had a set() since the last save (implementation may just save all).
struct ExternalBindings {
  bool (*get)(uint16_t extId, float& out) = nullptr;
  bool (*set)(uint16_t extId, float value) = nullptr;
  bool (*save)() = nullptr;
};

// One callback for "stored params in this group changed" — re-stage runtime.
using GroupHook = void (*)(uint8_t group);

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

// Open NVS, run stored-config migration, load stored params (defaults where
// keys are missing / invalid). Call once from setup() after Preferences is
// usable. Safe to call before registerExternalBindings(); external values
// simply read as their table defaults until bound.
bool begin();

void registerExternalBindings(const ExternalBindings& b);
void registerGroupHook(GroupHook hook);

// ---------------------------------------------------------------------------
// Introspection (used by the /api/config UI)
// ---------------------------------------------------------------------------
size_t paramCount();
const ParamDef* paramByIndex(size_t idx);
const ParamDef* paramByName(const char* name, size_t* idxOut = nullptr);

// Current live value (stored array or external binding). Returns false if the
// external binding is missing.
bool getValue(size_t idx, float& out);

// ---------------------------------------------------------------------------
// Mutation — all validated, all safe-context only
// ---------------------------------------------------------------------------

// Validate + apply ONE param to RAM and persist it. Fires the group hook.
// Returns false (and changes nothing) when out of range / unknown / NVS error.
bool setAndSave(const char* name, float value);

// Result detail for import/apply-many so the UI can say exactly what failed.
struct ImportResult {
  bool ok = false;
  uint16_t applied = 0;        // pairs applied
  uint16_t unknown = 0;        // pairs skipped: name not in table (forward compat)
  uint16_t rejected = 0;       // pairs failing range validation
  bool rolledBack = false;     // NVS failure mid-apply -> RAM restored
  uint16_t fileVersion = 0;    // config_version parsed from the document
  char firstError[48] = {};    // first offending param name (or reason)
};

// Parse a JSON document of the /api/config/export shape:
//   {"config_version":N, "params":{"name":value, ...}}
// Unknown names are counted and skipped. ANY out-of-range value rejects the
// whole document before anything is applied. On success every pair is applied
// to RAM, persisted, and group hooks fire once per touched group.
ImportResult importJson(const char* json, size_t len);

// Same validation/apply path for a bare {"name":value,...} object (the POST
// /api/config body). Treated as current-version data.
ImportResult applyJsonObject(const char* json, size_t len);

// Reset every STORED param to its table default, persist, fire all hooks.
// External params are intentionally untouched (they have their own reset
// endpoints and flight-critical ownership).
bool resetStoredToDefaults();

// ---------------------------------------------------------------------------
// Export — chunked so the HTTP layer can stream without a giant buffer.
// Writes up to maxLen bytes of the export JSON starting at byte offset
// `cursor`; returns bytes written and sets nextCursor (0 == done).
// The export is deterministic for a given config state.
// ---------------------------------------------------------------------------
uint32_t exportJsonChunk(uint32_t cursor, char* buf, uint32_t maxLen,
                         uint32_t& nextCursor);

// Meta+value listing for the config UI (array form with min/max/def/group).
uint32_t metaJsonChunk(uint32_t cursor, char* buf, uint32_t maxLen,
                       uint32_t& nextCursor);

// True while an import/apply is mutating state — consumed as an arming gate.
bool writeInProgress();

// Stored-config schema version currently in NVS (after begin()).
uint16_t storedVersion();

}  // namespace fcu_config
