/**
 * Beginner-friendly ESC class wrapper.
 *
 * Goal:
 * - Make usage simple for Arduino learners.
 * - Hide most setup details behind one class.
 */

#pragma once

#include "esc_controller.h"
#include <array>

namespace esc
{
struct EasyEscConfig
{
    uint8_t motorCount = kMaxMotors;
    std::array<gpio_num_t, kMaxMotors> motorPins = {GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC};
    std::array<bool, kMaxMotors> motorDirectionReversed = {false, false, false, false};
    std::array<uint16_t, kMaxMotors> motorTxBufferSymbols = {
        kDefaultRmtTxBufferSymbols,
        kDefaultRmtTxBufferSymbols,
        kDefaultRmtTxBufferSymbols,
        kDefaultRmtTxBufferSymbols};

    gpio_num_t currentPin = GPIO_NUM_NC; // Use GPIO_NUM_NC to disable current monitor.
    float currentZeroOffsetMv = 0.0f;
    float currentMvPerAmp = 33.0f;

    bool bidirectionalDshot = false;
    dshot_mode_t dshotMode = DSHOT300;
    uint32_t signalTimeoutMs = 20000;
    bool holdArmOnSignalTimeout = false;
    uint16_t outputRefreshMs = 10;

    // If true, begin() will automatically try fewer motors if RMT channels are limited.
    bool allowMotorCountFallback = true;
};

class EasyEsc
{
public:
    EasyEsc() = default;
    explicit EasyEsc(const EasyEscConfig &config);
    EasyEsc(
        uint8_t motorCount,
        gpio_num_t m1,
        gpio_num_t m2,
        gpio_num_t m3,
        gpio_num_t m4,
        gpio_num_t currentPin = GPIO_NUM_NC,
        dshot_mode_t mode = DSHOT300,
        uint32_t signalTimeoutMs = 20000,
        uint16_t outputRefreshMs = 10,
        float currentZeroOffsetMv = 0.0f,
        float currentMvPerAmp = 33.0f,
        bool m1Reversed = false,
        bool m2Reversed = false,
        bool m3Reversed = false,
        bool m4Reversed = false,
        bool bidirectionalDshot = false,
        uint16_t m1TxBufferSymbols = kDefaultRmtTxBufferSymbols,
        uint16_t m2TxBufferSymbols = kDefaultRmtTxBufferSymbols,
        uint16_t m3TxBufferSymbols = kDefaultRmtTxBufferSymbols,
        uint16_t m4TxBufferSymbols = kDefaultRmtTxBufferSymbols);

    bool begin();
    void update(); // call in loop()

    bool arm();
    bool disarm();
    bool stopAll();

    bool setMotorRaw(uint8_t motor, uint16_t raw);
    bool setMotorPercent(uint8_t motor, float percent); // 0..100
    bool sendMotorCommandFrame(uint8_t motor, uint16_t command); // one DShot cmd frame (1..47)
    bool setAllRaw(uint16_t raw);
    bool setAllPercent(float percent); // 0..100

    bool enterPassthrough(uint8_t motor);
    bool exitPassthrough();
    bool isInPassthrough() const;
    uint8_t activePassthroughMotor() const;
    uint8_t passthroughRead(uint16_t address, uint8_t *data, uint8_t length);
    uint8_t passthroughWrite(uint16_t address, const uint8_t *data, uint8_t length);
    uint8_t passthroughErase(uint16_t address);

    bool isArmed() const;
    bool isInitialized() const;

    void setTimeoutMs(uint32_t timeoutMs);
    uint32_t timeoutMs() const;
    void setHoldArmOnSignalTimeout(bool enabled);
    bool holdArmOnSignalTimeout() const;
    void setRefreshMs(uint16_t refreshMs);
    uint16_t refreshMs() const;

    bool hasCurrentMonitor() const;
    gpio_num_t currentPin() const;
    CurrentSample readCurrent() const;
    float currentAmps() const;
    float currentMilliamps() const;
    uint16_t currentMilliVolts() const;
    void setCurrentCalibration(float zeroOffsetMv, float mvPerAmp);
    bool calibrateCurrentZero(uint16_t samples = 200, uint16_t sampleDelayMs = 2);
    float currentZeroOffsetMv() const;
    float currentMvPerAmp() const;

    uint8_t motorCount() const;
    gpio_num_t motorPin(uint8_t motor) const;
    dshot_mode_t dshotMode() const;
    const char *dshotModeName() const;

    int32_t lastRmtErrorCode() const;
    int8_t lastRmtErrorMotor() const;
    gpio_num_t lastRmtErrorPin() const;
    MotorTelemetry motorTelemetry(uint8_t motor) const;
    // One-shot KISS/TLM-wire telemetry request for one motor's next frame.
    Status requestMotorUartTelemetry(uint8_t motor);

    Status lastStatus() const;
    uint8_t requestedMotorCount() const;

    static bool isMotorPinSuitable(gpio_num_t pin);
    static bool isCurrentPinSuitable(gpio_num_t pin);
    bool areConfiguredPinsSuitable() const;

private:
    EscControllerConfig makeControllerConfig(uint8_t motorCount) const;
    bool applyStatus(Status s);

    EasyEscConfig _cfg = {};
    EscController _ctrl = {};
    Status _lastStatus = Status::NotInitialized;
    uint8_t _activeMotorCount = 0;
};

/**
 * Servo-style single motor object.
 *
 * Example:
 *   esc::EasyEscMotor MOTOR1(GPIO_NUM_41);
 *   MOTOR1.begin();
 *   MOTOR1.arm();
 *   MOTOR1.spinPercent(20);
 */
class EasyEscMotor
{
public:
    EasyEscMotor(
        gpio_num_t motorPin,
        gpio_num_t currentPin = GPIO_NUM_NC,
        dshot_mode_t mode = DSHOT300,
        uint32_t signalTimeoutMs = 20000,
        uint16_t outputRefreshMs = 10,
        float currentZeroOffsetMv = 0.0f,
        float currentMvPerAmp = 33.0f,
        bool reversed = false,
        bool bidirectionalDshot = false,
        uint16_t txBufferSymbols = kDefaultRmtTxBufferSymbols);

    bool begin();
    void update();

    bool arm();
    bool disarm();
    bool stop();

    bool spinRaw(uint16_t raw);
    bool spinPercent(float percent); // 0..100
    // Send ONE DShot special command frame (1..47) to this motor (e.g. 3D-mode
    // off, save settings). Motors must be stopped; repeat ~10x spaced >= the
    // DShot frame interval for the ESC to latch it. See EscDshotOutput.
    bool sendDshotCommandFrame(uint16_t command);

    bool enterPassthrough();
    bool exitPassthrough();
    bool isInPassthrough() const;

    uint8_t passthroughRead(uint16_t address, uint8_t *data, uint8_t length);
    uint8_t passthroughWrite(uint16_t address, const uint8_t *data, uint8_t length);
    uint8_t passthroughErase(uint16_t address);

    bool isArmed() const;
    bool isInitialized() const;

    void setTimeoutMs(uint32_t timeoutMs);
    uint32_t timeoutMs() const;
    void setHoldArmOnSignalTimeout(bool enabled);
    bool holdArmOnSignalTimeout() const;
    void setRefreshMs(uint16_t refreshMs);
    uint16_t refreshMs() const;

    bool hasCurrentMonitor() const;
    gpio_num_t currentPin() const;
    CurrentSample readCurrent() const;
    float currentAmps() const;
    float currentMilliamps() const;
    uint16_t currentMilliVolts() const;
    void setCurrentCalibration(float zeroOffsetMv, float mvPerAmp);
    bool calibrateCurrentZero(uint16_t samples = 200, uint16_t sampleDelayMs = 2);
    float currentZeroOffsetMv() const;
    float currentMvPerAmp() const;

    gpio_num_t pin() const;
    dshot_mode_t dshotMode() const;
    const char *dshotModeName() const;

    int32_t lastRmtErrorCode() const;
    int8_t lastRmtErrorMotor() const;
    gpio_num_t lastRmtErrorPin() const;
    MotorTelemetry telemetry() const;
    // Arm a one-shot KISS/TLM-wire telemetry request on this motor's next
    // throttle frame. Returns true when the request was accepted.
    bool requestUartTelemetry();
    Status lastStatus() const;

    bool isMotorPinSuitable() const;
    bool isCurrentPinSuitable() const;
    bool arePinsSuitable() const;

private:
    EasyEsc _esc;
};
} // namespace esc
