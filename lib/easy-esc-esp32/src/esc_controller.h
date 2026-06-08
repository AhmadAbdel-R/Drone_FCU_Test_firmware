/**
 * High-level ESC library built on top of esc_dshot_output + esc_passthrough.
 *
 * Goals:
 * - Beginner-friendly API for arm/disarm/throttle control.
 * - Motor-object access (controller.motor(0).spinRaw(...)).
 * - Preserve passthrough access for BLHeli_S / Bluejay tooling.
 */

#pragma once

#include "esc_common.h"
#include "esc_dshot_output.h"
#include "esc_passthrough.h"
#include <array>

namespace esc
{
struct CurrentMonitorConfig
{
    bool enabled = false;
    gpio_num_t pin = GPIO_NUM_NC;
    float zeroOffsetMv = 0.0f;
    float mvPerAmp = 33.0f;
    uint16_t adcRefMv = 3300;
    uint16_t adcMaxRaw = 4095;
};

struct CurrentSample
{
    bool valid = false;
    uint16_t raw = 0;
    uint16_t milliVolts = 0;
    float amps = 0.0f;
};

struct EscControllerConfig
{
    uint8_t motorCount = kMaxMotors;
    std::array<gpio_num_t, kMaxMotors> motorPins = {GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC};
    std::array<bool, kMaxMotors> motorDirectionReversed = {false, false, false, false};
    std::array<uint16_t, kMaxMotors> motorTxBufferSymbols = {
        kDefaultRmtTxBufferSymbols,
        kDefaultRmtTxBufferSymbols,
        kDefaultRmtTxBufferSymbols,
        kDefaultRmtTxBufferSymbols};
    bool bidirectionalDshot = false;
    dshot_mode_t dshotMode = DSHOT300;
    uint16_t throttleMinRaw = kDshotThrottleMinRaw;
    uint16_t throttleMaxRaw = kDshotThrottleMaxRaw;
    uint32_t signalTimeoutMs = 20000;
    bool holdArmOnSignalTimeout = false;
    uint16_t outputRefreshMs = 10;
    CurrentMonitorConfig current = {};
    PassthroughConfig passthrough = {};
};

class EscController;

class EscMotor
{
public:
    EscMotor() = default;
    EscMotor(EscController *owner, uint8_t index);

    bool valid() const;
    uint8_t index() const;
    gpio_num_t pin() const;

    Status spinRaw(uint16_t raw);
    Status spinNormalized(float normalized); // 0.0 .. 1.0
    Status spinPercent(float percent);       // 0 .. 100
    Status stop();

private:
    EscController *_owner = nullptr;
    uint8_t _index = 0;
};

class EscController
{
public:
    EscController() = default;
    explicit EscController(const EscControllerConfig &config);
    EscController(
        uint8_t motorCount,
        const std::array<gpio_num_t, kMaxMotors> &motorPins,
        gpio_num_t currentPin = GPIO_NUM_NC,
        dshot_mode_t mode = DSHOT300,
        uint32_t signalTimeoutMs = 20000,
        uint16_t outputRefreshMs = 10,
        float currentZeroOffsetMv = 0.0f,
        float currentMvPerAmp = 33.0f);

    Status begin();
    Status begin(const EscControllerConfig &config);
    bool isInitialized() const;

    void service();

    Status arm();
    Status disarm();
    bool isArmed() const;

    Status setMotorRaw(uint8_t motor, uint16_t raw);
    Status setAllRaw(uint16_t raw);
    Status stopAll();

    void setSignalTimeoutMs(uint32_t timeoutMs);
    uint32_t signalTimeoutMs() const;
    void setHoldArmOnSignalTimeout(bool enabled);
    bool holdArmOnSignalTimeout() const;

    void setOutputRefreshMs(uint16_t refreshMs);
    uint16_t outputRefreshMs() const;

    uint8_t motorCount() const;
    gpio_num_t motorPin(uint8_t motor) const;
    dshot_mode_t dshotMode() const;
    const char *dshotModeName() const;
    bool hasCurrentMonitor() const;
    gpio_num_t currentPin() const;
    CurrentSample readCurrentSample() const;
    void setCurrentCalibration(float zeroOffsetMv, float mvPerAmp);
    float currentZeroOffsetMv() const;
    float currentMvPerAmp() const;

    EscMotor motor(uint8_t index);

    Status enterPassthroughMode(uint8_t motor);
    Status exitPassthroughMode();
    bool isInPassthroughMode() const;
    uint8_t activePassthroughMotor() const;

    uint8_t passthroughRead(uint16_t address, uint8_t *data, uint8_t length);
    uint8_t passthroughWrite(uint16_t address, const uint8_t *data, uint8_t length);
    uint8_t passthroughErase(uint16_t address);

    int32_t lastDriverErrorCode() const;
    int8_t lastDriverErrorMotor() const;
    gpio_num_t lastDriverErrorPin() const;
    MotorTelemetry motorTelemetry(uint8_t motor) const;

    EscDshotOutput &output();
    EscPassthrough &passthrough();

private:
    static uint16_t normalizedToRaw(float normalized, uint16_t minRaw, uint16_t maxRaw);
    static const char *modeName(dshot_mode_t mode);

    bool _initialized = false;
    EscControllerConfig _config = {};
    EscDshotOutput _output = {};
    EscPassthrough _passthrough = {};

    friend class EscMotor;
};
} // namespace esc
