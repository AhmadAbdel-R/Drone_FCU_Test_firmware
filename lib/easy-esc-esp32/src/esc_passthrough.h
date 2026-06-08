/**
 * Minimal BLHeli_S passthrough scaffold extracted from Flight32 MotorTask.
 *
 * WARNING:
 * - This preserves Flight32's experimental passthrough behavior.
 * - BLHeli_S command sequencing is still incomplete (see TODO markers in .cpp).
 */

#pragma once

#include "esc_common.h"
#include "esc_dshot_output.h"
#include <Arduino.h>
#include <cstdint>

namespace esc
{
class EscPassthrough
{
public:
    Status begin(EscDshotOutput *output, const PassthroughConfig &config = PassthroughConfig{});

    Status enterPassthroughMode(uint8_t motor);
    Status exitPassthroughMode();
    bool isInPassthroughMode() const;
    uint8_t activeMotor() const;

    uint8_t passthroughRead(uint8_t motorId, uint16_t address, uint8_t *data, uint8_t length);
    uint8_t passthroughWrite(uint8_t motorId, uint16_t address, const uint8_t *data, uint8_t length);
    uint8_t passthroughErase(uint8_t motorId, uint16_t address);

private:
    void oneWireOutputMode();
    void oneWireInputMode();
    void oneWireSetHigh();
    void oneWireSetLow();
    int oneWireReadPin();
    void oneWireHalfBitDelay();
    uint8_t oneWireReadByte();
    void oneWireSendByte(uint8_t byte);

    uint8_t oneWireTransaction(uint8_t cmd, uint16_t address, const uint8_t *writeData, uint8_t *readData, uint8_t length, bool isWrite);
    uint8_t performSimplifiedHandshake();

    EscDshotOutput *_output = nullptr;
    PassthroughConfig _config = {};
    bool _initialized = false;
    bool _inPassthroughMode = false;
    uint8_t _activeMotor = 0;
    gpio_num_t _passthroughGpio = GPIO_NUM_NC;
};
} // namespace esc
