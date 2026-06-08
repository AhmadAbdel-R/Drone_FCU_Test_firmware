/**
 * Minimal ESC subsystem extracted and refactored from Flight32.
 *
 * Original Flight32 sources used:
 * - src/tasks/motor_task.h
 * - src/tasks/motor_task.cpp
 * - src/protocols/serial_4way_protocol.h
 * - src/config/motor_config.h
 *
 * License attribution: MIT (see THIRD_PARTY_NOTICES.md).
 */

#pragma once

#include <DShotRMT.h>
#include <array>
#include <cstdint>
#include <driver/gpio.h>

namespace esc
{
constexpr uint8_t kMaxMotors = 4;
constexpr uint16_t kDshotThrottleMinRaw = 48;
constexpr uint16_t kDshotThrottleMaxRaw = 2047;
constexpr uint16_t kDefaultRmtTxBufferSymbols = static_cast<uint16_t>(RMT_TX_BUFFER_SYMBOLS);

enum class Status : uint8_t
{
    Ok = 0,
    InvalidArg,
    NotInitialized,
    NotArmed,
    PassthroughActive,
    NotInPassthroughMode,
    DriverError
};

struct DshotOutputConfig
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
    uint32_t signalTimeoutMs = 200;
    uint16_t outputRefreshMs = 10; // Periodic resend of last throttle values while armed.
    bool holdArmOnSignalTimeout = false; // If true: on timeout force zero throttle but keep armed.
};

struct MotorTelemetry
{
    bool valid = false;
    uint32_t erpm = 0;
    uint32_t motorRpm = 0;
    uint32_t goodFrameCount = 0;
    uint32_t badFrameCount = 0;
    uint32_t lastUpdateMs = 0;
};

struct PassthroughTimingConfig
{
    uint16_t resetUs = 650;
    uint16_t delayUs = 10;
    uint16_t halfBitDelayUs = 14;
    uint16_t startUs = 5;
    uint16_t stopUs = 10;
    uint16_t byteDelayUs = 200;
};

struct PassthroughConfig
{
    PassthroughTimingConfig timing = {};
};

// Betaflight-style 4-way serial constants (from Flight32 serial_4way_protocol.h).
constexpr uint8_t kCmdRemoteEscape = 0x2E; // '.'
constexpr uint8_t kCmdLocalEscape = 0x2F;  // '/'

constexpr uint8_t kCmdInterfaceTestAlive = 0x30;
constexpr uint8_t kCmdProtocolGetVersion = 0x31;
constexpr uint8_t kCmdInterfaceGetName = 0x32;
constexpr uint8_t kCmdInterfaceGetVersion = 0x33;
constexpr uint8_t kCmdInterfaceExit = 0x34;
constexpr uint8_t kCmdDeviceReset = 0x35;
constexpr uint8_t kCmdDeviceInitFlash = 0x37;
constexpr uint8_t kCmdDeviceEraseAll = 0x38;
constexpr uint8_t kCmdDevicePageErase = 0x39;
constexpr uint8_t kCmdDeviceRead = 0x3A;
constexpr uint8_t kCmdDeviceWrite = 0x3B;
constexpr uint8_t kCmdDeviceC2ckLow = 0x3C;
constexpr uint8_t kCmdDeviceReadEeprom = 0x3D;
constexpr uint8_t kCmdDeviceWriteEeprom = 0x3E;
constexpr uint8_t kCmdInterfaceSetMode = 0x3F;
constexpr uint8_t kCmdDeviceVerify = 0x40;

// BLHeli_S bootloader command constants (from Flight32 serial_4way_protocol.h).
constexpr uint8_t kBlReadVersion = 0x02;
constexpr uint8_t kBlEraseCodePage = 0x04;
constexpr uint8_t kBlWriteCodeWord = 0x06;
constexpr uint8_t kBlGetWord = 0x0A;
constexpr uint8_t kBlGetId = 0x0B;
constexpr uint8_t kBlGo = 0x0C;
constexpr uint8_t kBlWriteDataBlock = 0x0D;
constexpr uint8_t kBlReadDataBlock = 0x0E;
constexpr uint8_t kBlGetChipId = 0x0F;

// ACK codes.
constexpr uint8_t kAckOk = 0x00;
constexpr uint8_t kAckBootloader = 0x01;
constexpr uint8_t kNackBootloader = 0x00;
constexpr uint8_t kAckInvalidCmd = 0x02;
constexpr uint8_t kAckInvalidCrc = 0x03;
constexpr uint8_t kAckVerifyError = 0x04;
constexpr uint8_t kAckInvalidChannel = 0x08;
constexpr uint8_t kAckInvalidParam = 0x09;
constexpr uint8_t kAckGeneralError = 0x0F;

constexpr uint8_t kPassthroughStartByte = 0x08;
} // namespace esc
