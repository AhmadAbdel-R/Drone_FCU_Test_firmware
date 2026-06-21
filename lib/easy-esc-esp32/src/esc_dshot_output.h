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

    // Arm a one-shot UART (KISS/BLHeli32 TLM-wire) telemetry request on this
    // motor's next throttle frame. See DShotRMT::requestTelemetrySignal().
    Status requestMotorUartTelemetry(uint8_t motor);

    // Required by passthrough mode to release/re-acquire the selected motor pin.
    Status suspendMotorDriverForPassthrough(uint8_t motor);
    Status resumeMotorDriverFromPassthrough(uint8_t motor);

    // Transmit ONE DShot special command frame (values 1..47) on this motor.
    // Used to send config commands (e.g. 3D-mode off, save settings) over the
    // normal DShot wire. The caller is responsible for stopping the motors,
    // spacing the calls (>= the DShot frame interval) and repeating enough times
    // (AM32/BLHeli require ~6+ consecutive) for the ESC to latch the setting.
    Status sendMotorCommandFrame(uint8_t motor, uint16_t command);

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
