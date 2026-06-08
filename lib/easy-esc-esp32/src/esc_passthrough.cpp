#include "esc_passthrough.h"

namespace esc
{
Status EscPassthrough::begin(EscDshotOutput *output, const PassthroughConfig &config)
{
    if (!output || !output->isInitialized())
    {
        return Status::NotInitialized;
    }

    _output = output;
    _config = config;
    _initialized = true;
    _inPassthroughMode = false;
    _activeMotor = 0;
    _passthroughGpio = GPIO_NUM_NC;
    return Status::Ok;
}

Status EscPassthrough::enterPassthroughMode(uint8_t motor)
{
    if (!_initialized || !_output)
    {
        return Status::NotInitialized;
    }
    if (motor >= _output->motorCount())
    {
        return Status::InvalidArg;
    }

    if (_inPassthroughMode)
    {
        if (_activeMotor == motor)
        {
            return Status::Ok;
        }
        const Status exitStatus = exitPassthroughMode();
        if (exitStatus != Status::Ok)
        {
            return exitStatus;
        }
    }

    Status status = _output->setPassthroughActive(true);
    if (status != Status::Ok)
    {
        return status;
    }

    status = _output->suspendMotorDriverForPassthrough(motor);
    if (status != Status::Ok)
    {
        _output->setPassthroughActive(false);
        return status;
    }

    _passthroughGpio = _output->motorPin(motor);
    if (_passthroughGpio == GPIO_NUM_NC)
    {
        _output->resumeMotorDriverFromPassthrough(motor);
        _output->setPassthroughActive(false);
        return Status::InvalidArg;
    }

    _activeMotor = motor;
    _inPassthroughMode = true;

    gpio_reset_pin(_passthroughGpio);
    gpio_set_direction(_passthroughGpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_passthroughGpio, GPIO_PULLUP_ONLY);

    return Status::Ok;
}

Status EscPassthrough::exitPassthroughMode()
{
    if (!_initialized || !_output)
    {
        return Status::NotInitialized;
    }
    if (!_inPassthroughMode)
    {
        return Status::NotInPassthroughMode;
    }

    Status status = _output->resumeMotorDriverFromPassthrough(_activeMotor);
    if (status != Status::Ok)
    {
        return status;
    }

    status = _output->setPassthroughActive(false);
    if (status != Status::Ok)
    {
        return status;
    }

    _inPassthroughMode = false;
    _passthroughGpio = GPIO_NUM_NC;
    return Status::Ok;
}

bool EscPassthrough::isInPassthroughMode() const
{
    return _inPassthroughMode;
}

uint8_t EscPassthrough::activeMotor() const
{
    return _activeMotor;
}

uint8_t EscPassthrough::passthroughRead(uint8_t motorId, uint16_t address, uint8_t *data, uint8_t length)
{
    if (!_inPassthroughMode)
    {
        return kAckGeneralError;
    }
    if (motorId != _activeMotor)
    {
        return kAckInvalidChannel;
    }

    const uint8_t handshakeAck = performSimplifiedHandshake();
    if (handshakeAck != kAckOk)
    {
        return handshakeAck;
    }

    // TODO(Flight32 extraction): Full BLHeli_S command flow is not implemented.
    // This keeps Flight32's placeholder READ_DATA_BLOCK transaction scaffold.
    return oneWireTransaction(kBlReadDataBlock, address, nullptr, data, length, false);
}

uint8_t EscPassthrough::passthroughWrite(uint8_t motorId, uint16_t address, const uint8_t *data, uint8_t length)
{
    if (!_inPassthroughMode)
    {
        return kAckGeneralError;
    }
    if (motorId != _activeMotor)
    {
        return kAckInvalidChannel;
    }

    const uint8_t handshakeAck = performSimplifiedHandshake();
    if (handshakeAck != kAckOk)
    {
        return handshakeAck;
    }

    // TODO(Flight32 extraction): Full BLHeli_S command flow is not implemented.
    // This keeps Flight32's placeholder WRITE_DATA_BLOCK transaction scaffold.
    return oneWireTransaction(kBlWriteDataBlock, address, data, nullptr, length, true);
}

uint8_t EscPassthrough::passthroughErase(uint8_t motorId, uint16_t address)
{
    if (!_inPassthroughMode)
    {
        return kAckGeneralError;
    }
    if (motorId != _activeMotor)
    {
        return kAckInvalidChannel;
    }

    const uint8_t handshakeAck = performSimplifiedHandshake();
    if (handshakeAck != kAckOk)
    {
        return handshakeAck;
    }

    // TODO(Flight32 extraction): Full BLHeli_S erase semantics are incomplete.
    // This keeps Flight32's placeholder ERASE_CODE_PAGE transaction scaffold.
    return oneWireTransaction(kBlEraseCodePage, address, nullptr, nullptr, 0, true);
}

void EscPassthrough::oneWireOutputMode()
{
    gpio_set_direction(_passthroughGpio, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(_passthroughGpio, GPIO_FLOATING);
}

void EscPassthrough::oneWireInputMode()
{
    gpio_set_direction(_passthroughGpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(_passthroughGpio, GPIO_PULLUP_ONLY);
}

void EscPassthrough::oneWireSetHigh()
{
    gpio_set_level(_passthroughGpio, 1);
}

void EscPassthrough::oneWireSetLow()
{
    gpio_set_level(_passthroughGpio, 0);
}

int EscPassthrough::oneWireReadPin()
{
    return gpio_get_level(_passthroughGpio);
}

void EscPassthrough::oneWireHalfBitDelay()
{
    delayMicroseconds(_config.timing.halfBitDelayUs);
}

uint8_t EscPassthrough::oneWireReadByte()
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {
        oneWireInputMode();
        oneWireHalfBitDelay();
        data >>= 1;
        if (oneWireReadPin() == 1)
        {
            data |= 0x80;
        }
        oneWireHalfBitDelay();
    }
    return data;
}

void EscPassthrough::oneWireSendByte(uint8_t byte)
{
    oneWireOutputMode();
    for (uint8_t i = 0; i < 8; ++i)
    {
        if (byte & 0x01)
        {
            oneWireSetHigh();
        }
        else
        {
            oneWireSetLow();
        }
        oneWireHalfBitDelay();
        oneWireSetLow();
        oneWireHalfBitDelay();
        byte >>= 1;
    }
}

uint8_t EscPassthrough::oneWireTransaction(uint8_t cmd, uint16_t address, const uint8_t *writeData, uint8_t *readData, uint8_t length, bool isWrite)
{
    uint8_t checksum = 0;

    oneWireSendByte(kPassthroughStartByte);
    checksum = static_cast<uint8_t>(checksum + kPassthroughStartByte);

    uint8_t payloadLength = isWrite ? static_cast<uint8_t>(2 + length) : 2;
    oneWireSendByte(payloadLength);
    checksum = static_cast<uint8_t>(checksum + payloadLength);

    oneWireSendByte(cmd);
    checksum = static_cast<uint8_t>(checksum + cmd);

    oneWireSendByte(static_cast<uint8_t>(address >> 8));
    checksum = static_cast<uint8_t>(checksum + static_cast<uint8_t>(address >> 8));

    oneWireSendByte(static_cast<uint8_t>(address & 0xFF));
    checksum = static_cast<uint8_t>(checksum + static_cast<uint8_t>(address & 0xFF));

    if (isWrite && writeData && length > 0)
    {
        for (uint8_t i = 0; i < length; ++i)
        {
            oneWireSendByte(writeData[i]);
            checksum = static_cast<uint8_t>(checksum + writeData[i]);
        }
    }

    oneWireSendByte(checksum);

    uint8_t rxStartByte = oneWireReadByte();
    uint8_t rxLength = oneWireReadByte();

    // TODO(Flight32 extraction): Validate rxStartByte/rxLength according to full BLHeli_S protocol.
    // TODO(Flight32 extraction): Validate and enforce full CRC/checksum rules (request + response).
    (void)rxStartByte;

    for (uint8_t i = 0; i < rxLength; ++i)
    {
        const uint8_t value = oneWireReadByte();
        if (!isWrite && readData && i < length)
        {
            readData[i] = value;
        }
    }

    const uint8_t responseAck = oneWireReadByte();
    uint8_t responseChecksum = oneWireReadByte();
    (void)responseChecksum;

    return responseAck;
}

uint8_t EscPassthrough::performSimplifiedHandshake()
{
    // TODO(Flight32 extraction): This is a simplified placeholder handshake copied from Flight32.
    // The complete BLHeli_S bootloader connect/init sequence is not yet implemented.
    oneWireOutputMode();
    oneWireSetHigh();
    delayMicroseconds(_config.timing.resetUs);
    oneWireInputMode();
    oneWireHalfBitDelay();

    const uint8_t resetResponse = oneWireReadByte();
    if (resetResponse != 0x00)
    {
        return kAckGeneralError;
    }

    oneWireSendByte(0x30); // Simplified "connect" command from Flight32 placeholder.
    const uint8_t connectResponse = oneWireReadByte();
    if (connectResponse != 0xCC)
    {
        return kAckGeneralError;
    }

    return kAckOk;
}
} // namespace esc
