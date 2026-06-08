#include "esc_controller.h"

#include <Arduino.h>

namespace esc
{
EscMotor::EscMotor(EscController *owner, uint8_t index)
    : _owner(owner), _index(index)
{
}

bool EscMotor::valid() const
{
    return _owner != nullptr && _index < _owner->motorCount();
}

uint8_t EscMotor::index() const
{
    return _index;
}

gpio_num_t EscMotor::pin() const
{
    if (!valid())
    {
        return GPIO_NUM_NC;
    }
    return _owner->motorPin(_index);
}

Status EscMotor::spinRaw(uint16_t raw)
{
    if (!valid())
    {
        return Status::InvalidArg;
    }
    return _owner->setMotorRaw(_index, raw);
}

Status EscMotor::spinNormalized(float normalized)
{
    if (!valid())
    {
        return Status::InvalidArg;
    }

    const uint16_t raw = EscController::normalizedToRaw(
        normalized,
        _owner->_config.throttleMinRaw,
        _owner->_config.throttleMaxRaw);
    return _owner->setMotorRaw(_index, raw);
}

Status EscMotor::spinPercent(float percent)
{
    return spinNormalized(percent / 100.0f);
}

Status EscMotor::stop()
{
    return spinRaw(0);
}

EscController::EscController(const EscControllerConfig &config)
    : _config(config)
{
}

EscController::EscController(
    uint8_t motorCount,
    const std::array<gpio_num_t, kMaxMotors> &motorPins,
    gpio_num_t currentPin,
    dshot_mode_t mode,
    uint32_t signalTimeoutMs,
    uint16_t outputRefreshMs,
    float currentZeroOffsetMv,
    float currentMvPerAmp)
{
    _config.motorCount = motorCount;
    _config.motorPins = motorPins;
    _config.dshotMode = mode;
    _config.signalTimeoutMs = signalTimeoutMs;
    _config.outputRefreshMs = outputRefreshMs;
    _config.current.enabled = (currentPin != GPIO_NUM_NC);
    _config.current.pin = currentPin;
    _config.current.zeroOffsetMv = currentZeroOffsetMv;
    _config.current.mvPerAmp = currentMvPerAmp;
}

Status EscController::begin()
{
    return begin(_config);
}

Status EscController::begin(const EscControllerConfig &config)
{
    _config = config;

    DshotOutputConfig lowConfig;
    lowConfig.motorCount = _config.motorCount;
    lowConfig.motorPins = _config.motorPins;
    lowConfig.motorDirectionReversed = _config.motorDirectionReversed;
    lowConfig.motorTxBufferSymbols = _config.motorTxBufferSymbols;
    lowConfig.bidirectionalDshot = _config.bidirectionalDshot;
    lowConfig.dshotMode = _config.dshotMode;
    lowConfig.throttleMinRaw = _config.throttleMinRaw;
    lowConfig.throttleMaxRaw = _config.throttleMaxRaw;
    lowConfig.signalTimeoutMs = _config.signalTimeoutMs;
    lowConfig.holdArmOnSignalTimeout = _config.holdArmOnSignalTimeout;
    lowConfig.outputRefreshMs = _config.outputRefreshMs;

    const Status outStatus = _output.begin(lowConfig);
    if (outStatus != Status::Ok)
    {
        _initialized = false;
        return outStatus;
    }

    const Status passStatus = _passthrough.begin(&_output, _config.passthrough);
    if (passStatus != Status::Ok)
    {
        _initialized = false;
        return passStatus;
    }

    _config.motorCount = _output.motorCount();

    if (hasCurrentMonitor())
    {
        analogReadResolution(12);
#if defined(ADC_11db)
        analogSetPinAttenuation(static_cast<uint8_t>(_config.current.pin), ADC_11db);
#endif
        pinMode(static_cast<int>(_config.current.pin), INPUT);
    }

    _initialized = true;
    return Status::Ok;
}

bool EscController::isInitialized() const
{
    return _initialized;
}

void EscController::service()
{
    _output.service();
}

Status EscController::arm()
{
    return _output.arm();
}

Status EscController::disarm()
{
    return _output.disarm();
}

bool EscController::isArmed() const
{
    return _output.isArmed();
}

Status EscController::setMotorRaw(uint8_t motor, uint16_t raw)
{
    return _output.setMotorThrottleRaw(motor, raw);
}

Status EscController::setAllRaw(uint16_t raw)
{
    return _output.setAllMotorsThrottleRaw(raw);
}

Status EscController::stopAll()
{
    return _output.stopAllMotors();
}

void EscController::setSignalTimeoutMs(uint32_t timeoutMs)
{
    _output.setSignalTimeoutMs(timeoutMs);
    _config.signalTimeoutMs = timeoutMs;
}

uint32_t EscController::signalTimeoutMs() const
{
    return _output.signalTimeoutMs();
}

void EscController::setHoldArmOnSignalTimeout(bool enabled)
{
    _output.setHoldArmOnSignalTimeout(enabled);
    _config.holdArmOnSignalTimeout = enabled;
}

bool EscController::holdArmOnSignalTimeout() const
{
    return _output.holdArmOnSignalTimeout();
}

void EscController::setOutputRefreshMs(uint16_t refreshMs)
{
    _output.setOutputRefreshMs(refreshMs);
    _config.outputRefreshMs = refreshMs;
}

uint16_t EscController::outputRefreshMs() const
{
    return _output.outputRefreshMs();
}

uint8_t EscController::motorCount() const
{
    return _output.motorCount();
}

gpio_num_t EscController::motorPin(uint8_t motor) const
{
    return _output.motorPin(motor);
}

dshot_mode_t EscController::dshotMode() const
{
    return _output.dshotMode();
}

const char *EscController::dshotModeName() const
{
    return modeName(_output.dshotMode());
}

bool EscController::hasCurrentMonitor() const
{
    return _config.current.enabled && _config.current.pin != GPIO_NUM_NC;
}

gpio_num_t EscController::currentPin() const
{
    return _config.current.pin;
}

CurrentSample EscController::readCurrentSample() const
{
    CurrentSample sample;
    if (!hasCurrentMonitor())
    {
        return sample;
    }

    sample.valid = true;
    const uint16_t adcMax = (_config.current.adcMaxRaw == 0) ? 4095 : _config.current.adcMaxRaw;
    const uint16_t adcRef = (_config.current.adcRefMv == 0) ? 3300 : _config.current.adcRefMv;

    const int raw = analogRead(static_cast<int>(_config.current.pin));
    sample.raw = static_cast<uint16_t>(constrain(raw, 0, static_cast<int>(adcMax)));

    int mv = analogReadMilliVolts(static_cast<int>(_config.current.pin));
    if (mv <= 0)
    {
        mv = static_cast<int>((static_cast<uint32_t>(sample.raw) * adcRef) / adcMax);
    }
    sample.milliVolts = static_cast<uint16_t>(constrain(mv, 0, static_cast<int>(adcRef)));

    if (_config.current.mvPerAmp > 0.0f)
    {
        float amps = (static_cast<float>(sample.milliVolts) - _config.current.zeroOffsetMv) / _config.current.mvPerAmp;
        if (amps < 0.0f)
        {
            amps = 0.0f;
        }
        sample.amps = amps;
    }

    return sample;
}

void EscController::setCurrentCalibration(float zeroOffsetMv, float mvPerAmp)
{
    _config.current.zeroOffsetMv = zeroOffsetMv;
    if (mvPerAmp > 0.0f)
    {
        _config.current.mvPerAmp = mvPerAmp;
    }
}

float EscController::currentZeroOffsetMv() const
{
    return _config.current.zeroOffsetMv;
}

float EscController::currentMvPerAmp() const
{
    return _config.current.mvPerAmp;
}

EscMotor EscController::motor(uint8_t index)
{
    return EscMotor(this, index);
}

Status EscController::enterPassthroughMode(uint8_t motor)
{
    return _passthrough.enterPassthroughMode(motor);
}

Status EscController::exitPassthroughMode()
{
    return _passthrough.exitPassthroughMode();
}

bool EscController::isInPassthroughMode() const
{
    return _passthrough.isInPassthroughMode();
}

uint8_t EscController::activePassthroughMotor() const
{
    return _passthrough.activeMotor();
}

uint8_t EscController::passthroughRead(uint16_t address, uint8_t *data, uint8_t length)
{
    return _passthrough.passthroughRead(_passthrough.activeMotor(), address, data, length);
}

uint8_t EscController::passthroughWrite(uint16_t address, const uint8_t *data, uint8_t length)
{
    return _passthrough.passthroughWrite(_passthrough.activeMotor(), address, data, length);
}

uint8_t EscController::passthroughErase(uint16_t address)
{
    return _passthrough.passthroughErase(_passthrough.activeMotor(), address);
}

int32_t EscController::lastDriverErrorCode() const
{
    return _output.lastDriverErrorCode();
}

int8_t EscController::lastDriverErrorMotor() const
{
    return _output.lastDriverErrorMotor();
}

gpio_num_t EscController::lastDriverErrorPin() const
{
    return _output.lastDriverErrorPin();
}

MotorTelemetry EscController::motorTelemetry(uint8_t motor) const
{
    return _output.motorTelemetry(motor);
}

EscDshotOutput &EscController::output()
{
    return _output;
}

EscPassthrough &EscController::passthrough()
{
    return _passthrough;
}

uint16_t EscController::normalizedToRaw(float normalized, uint16_t minRaw, uint16_t maxRaw)
{
    if (normalized <= 0.0f)
    {
        return 0;
    }
    if (normalized >= 1.0f)
    {
        return maxRaw;
    }

    const float span = static_cast<float>(maxRaw - minRaw);
    const float raw = static_cast<float>(minRaw) + (normalized * span);
    if (raw <= static_cast<float>(minRaw))
    {
        return minRaw;
    }
    if (raw >= static_cast<float>(maxRaw))
    {
        return maxRaw;
    }
    return static_cast<uint16_t>(raw);
}

const char *EscController::modeName(dshot_mode_t mode)
{
    switch (mode)
    {
    case DSHOT150:
        return "DSHOT150";
    case DSHOT300:
        return "DSHOT300";
    case DSHOT600:
        return "DSHOT600";
    case DSHOT1200:
        return "DSHOT1200";
    default:
        return "DSHOT_UNKNOWN";
    }
}
} // namespace esc
