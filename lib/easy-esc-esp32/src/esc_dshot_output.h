/**
 * Minimal DShot motor output extracted from Flight32 MotorTask.
 * Safety behavior was tightened for standalone use.
 */

#pragma once

#include "esc_common.h"
#include <Arduino.h>
#include <array>
#include <memory>

namespace esc
{
class EscDshotOutput
{
public:
    Status begin(const DshotOutputConfig &config);
    bool isInitialized() const;

    Status arm();
    Status disarm();
    bool isArmed() const;

    Status setMotorThrottleRaw(uint8_t motor, uint16_t valueRaw);
    Status setAllMotorsThrottleRaw(uint16_t valueRaw);
    Status stopAllMotors();
    void service();

    Status setPassthroughActive(bool active);
    bool isPassthroughActive() const;

    uint8_t motorCount() const;
    gpio_num_t motorPin(uint8_t motor) const;
    dshot_mode_t dshotMode() const;
    void setSignalTimeoutMs(uint32_t timeoutMs);
    uint32_t signalTimeoutMs() const;
    void setHoldArmOnSignalTimeout(bool enabled);
    bool holdArmOnSignalTimeout() const;
    void setOutputRefreshMs(uint16_t refreshMs);
    uint16_t outputRefreshMs() const;
    int32_t lastDriverErrorCode() const;
    int8_t lastDriverErrorMotor() const;
    gpio_num_t lastDriverErrorPin() const;
    MotorTelemetry motorTelemetry(uint8_t motor) const;

    // Required by passthrough mode to release/re-acquire the selected motor pin.
    Status suspendMotorDriverForPassthrough(uint8_t motor);
    Status resumeMotorDriverFromPassthrough(uint8_t motor);

private:
    static bool isConfigValid(const DshotOutputConfig &config);
    static uint16_t clampRawThrottle(uint16_t valueRaw, const DshotOutputConfig &config);

    Status createDriver(uint8_t motor);
    void clearDriver(uint8_t motor);
    void pollMotorTelemetry(uint8_t motor);
    Status sendMotorRaw(uint8_t motor, uint16_t valueRaw);
    Status sendAllStored();

    DshotOutputConfig _config = {};
    std::array<std::unique_ptr<DShotRMT>, kMaxMotors> _drivers = {};
    std::array<uint16_t, kMaxMotors> _lastThrottleRaw = {0, 0, 0, 0};
    std::array<bool, kMaxMotors> _driverSuspended = {false, false, false, false};
    std::array<MotorTelemetry, kMaxMotors> _telemetry = {};

    bool _initialized = false;
    bool _armed = false;
    bool _passthroughActive = false;
    uint32_t _lastUpdateMs = 0;
    uint32_t _lastRefreshMs = 0;
    int32_t _lastDriverErrorCode = 0;
    int8_t _lastDriverErrorMotor = -1;
    gpio_num_t _lastDriverErrorPin = GPIO_NUM_NC;
};
} // namespace esc
