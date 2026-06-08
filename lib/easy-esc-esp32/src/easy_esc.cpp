#include "easy_esc.h"
#include <Arduino.h>

#if __has_include(<esp_adc/adc_oneshot.h>)
#include <esp_adc/adc_oneshot.h>
#define EASY_ESC_HAS_ADC_ONESHOT 1
#else
#define EASY_ESC_HAS_ADC_ONESHOT 0
#endif

namespace
{
bool isMotorPinSuitableImpl(gpio_num_t pin)
{
    if (pin == GPIO_NUM_NC)
    {
        return false;
    }
    return GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

bool isCurrentPinSuitableImpl(gpio_num_t pin)
{
    if (pin == GPIO_NUM_NC)
    {
        return true;
    }
    if (!GPIO_IS_VALID_GPIO(pin))
    {
        return false;
    }

#if EASY_ESC_HAS_ADC_ONESHOT
    adc_unit_t unit = ADC_UNIT_1;
    adc_channel_t channel = ADC_CHANNEL_0;
    return adc_oneshot_io_to_channel(static_cast<int>(pin), &unit, &channel) == ESP_OK;
#else
#if defined(CONFIG_IDF_TARGET_ESP32S3)
    const int p = static_cast<int>(pin);
    return p >= 1 && p <= 20;
#else
    return true;
#endif
#endif
}
} // namespace

namespace esc
{
EasyEsc::EasyEsc(const EasyEscConfig &config)
    : _cfg(config)
{
}

EasyEsc::EasyEsc(
    uint8_t motorCount,
    gpio_num_t m1,
    gpio_num_t m2,
    gpio_num_t m3,
    gpio_num_t m4,
    gpio_num_t currentPin,
    dshot_mode_t mode,
    uint32_t signalTimeoutMs,
    uint16_t outputRefreshMs,
    float currentZeroOffsetMv,
    float currentMvPerAmp,
    bool m1Reversed,
    bool m2Reversed,
    bool m3Reversed,
    bool m4Reversed,
    bool bidirectionalDshot,
    uint16_t m1TxBufferSymbols,
    uint16_t m2TxBufferSymbols,
    uint16_t m3TxBufferSymbols,
    uint16_t m4TxBufferSymbols)
{
    _cfg.motorCount = motorCount;
    _cfg.motorPins = {m1, m2, m3, m4};
    _cfg.motorDirectionReversed = {m1Reversed, m2Reversed, m3Reversed, m4Reversed};
    _cfg.motorTxBufferSymbols = {m1TxBufferSymbols, m2TxBufferSymbols, m3TxBufferSymbols, m4TxBufferSymbols};
    _cfg.bidirectionalDshot = bidirectionalDshot;
    _cfg.currentPin = currentPin;
    _cfg.currentZeroOffsetMv = currentZeroOffsetMv;
    _cfg.currentMvPerAmp = currentMvPerAmp;
    _cfg.dshotMode = mode;
    _cfg.signalTimeoutMs = signalTimeoutMs;
    _cfg.outputRefreshMs = outputRefreshMs;
}

bool EasyEsc::begin()
{
    _activeMotorCount = 0;
    _lastStatus = Status::DriverError;

    const int requested = constrain(static_cast<int>(_cfg.motorCount), 1, static_cast<int>(kMaxMotors));
    for (int i = 0; i < requested; ++i)
    {
        const gpio_num_t pin = _cfg.motorPins[static_cast<size_t>(i)];
        if (pin == GPIO_NUM_NC)
        {
            _lastStatus = Status::InvalidArg;
            return false;
        }
        if (!isMotorPinSuitable(pin))
        {
            Serial.printf("[EASY_ESC][WARN] motor pin suitability check failed for GPIO%d; trying init anyway.\n",
                          static_cast<int>(pin));
        }
    }
    if (!isCurrentPinSuitable(_cfg.currentPin))
    {
        Serial.printf("[EASY_ESC][WARN] current pin suitability check failed for GPIO%d; trying init anyway.\n",
                      static_cast<int>(_cfg.currentPin));
    }

    int startCount = requested;
    int minCount = _cfg.allowMotorCountFallback ? 1 : requested;

    for (int count = startCount; count >= minCount; --count)
    {
        _lastStatus = _ctrl.begin(makeControllerConfig(static_cast<uint8_t>(count)));
        if (_lastStatus == Status::Ok)
        {
            _activeMotorCount = _ctrl.motorCount();
            return true;
        }
    }

    return false;
}

void EasyEsc::update()
{
    _ctrl.service();
}

bool EasyEsc::arm()
{
    return applyStatus(_ctrl.arm());
}

bool EasyEsc::disarm()
{
    return applyStatus(_ctrl.disarm());
}

bool EasyEsc::stopAll()
{
    return applyStatus(_ctrl.stopAll());
}

bool EasyEsc::setMotorRaw(uint8_t motor, uint16_t raw)
{
    return applyStatus(_ctrl.setMotorRaw(motor, raw));
}

bool EasyEsc::setMotorPercent(uint8_t motor, float percent)
{
    return applyStatus(_ctrl.motor(motor).spinPercent(percent));
}

bool EasyEsc::setAllRaw(uint16_t raw)
{
    return applyStatus(_ctrl.setAllRaw(raw));
}

bool EasyEsc::setAllPercent(float percent)
{
    if (percent < 0.0f || percent > 100.0f)
    {
        _lastStatus = Status::InvalidArg;
        return false;
    }

    const float normalized = percent / 100.0f;
    uint16_t raw = 0;
    if (normalized > 0.0f)
    {
        const uint16_t minRaw = kDshotThrottleMinRaw;
        const uint16_t maxRaw = kDshotThrottleMaxRaw;
        raw = static_cast<uint16_t>(minRaw + ((maxRaw - minRaw) * normalized));
        if (raw < minRaw)
        {
            raw = minRaw;
        }
        if (raw > maxRaw)
        {
            raw = maxRaw;
        }
    }
    return setAllRaw(raw);
}

bool EasyEsc::enterPassthrough(uint8_t motor)
{
    return applyStatus(_ctrl.enterPassthroughMode(motor));
}

bool EasyEsc::exitPassthrough()
{
    return applyStatus(_ctrl.exitPassthroughMode());
}

bool EasyEsc::isInPassthrough() const
{
    return _ctrl.isInPassthroughMode();
}

uint8_t EasyEsc::activePassthroughMotor() const
{
    return _ctrl.activePassthroughMotor();
}

uint8_t EasyEsc::passthroughRead(uint16_t address, uint8_t *data, uint8_t length)
{
    return _ctrl.passthroughRead(address, data, length);
}

uint8_t EasyEsc::passthroughWrite(uint16_t address, const uint8_t *data, uint8_t length)
{
    return _ctrl.passthroughWrite(address, data, length);
}

uint8_t EasyEsc::passthroughErase(uint16_t address)
{
    return _ctrl.passthroughErase(address);
}

bool EasyEsc::isArmed() const
{
    return _ctrl.isArmed();
}

bool EasyEsc::isInitialized() const
{
    return _ctrl.isInitialized();
}

void EasyEsc::setTimeoutMs(uint32_t timeoutMs)
{
    _ctrl.setSignalTimeoutMs(timeoutMs);
    _cfg.signalTimeoutMs = timeoutMs;
}

uint32_t EasyEsc::timeoutMs() const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.signalTimeoutMs();
    }
    return _cfg.signalTimeoutMs;
}

void EasyEsc::setHoldArmOnSignalTimeout(bool enabled)
{
    _ctrl.setHoldArmOnSignalTimeout(enabled);
    _cfg.holdArmOnSignalTimeout = enabled;
}

bool EasyEsc::holdArmOnSignalTimeout() const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.holdArmOnSignalTimeout();
    }
    return _cfg.holdArmOnSignalTimeout;
}

void EasyEsc::setRefreshMs(uint16_t refreshMs)
{
    _ctrl.setOutputRefreshMs(refreshMs);
    _cfg.outputRefreshMs = refreshMs;
}

uint16_t EasyEsc::refreshMs() const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.outputRefreshMs();
    }
    return _cfg.outputRefreshMs;
}

bool EasyEsc::hasCurrentMonitor() const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.hasCurrentMonitor();
    }
    return _cfg.currentPin != GPIO_NUM_NC;
}

gpio_num_t EasyEsc::currentPin() const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.currentPin();
    }
    return _cfg.currentPin;
}

CurrentSample EasyEsc::readCurrent() const
{
    return _ctrl.readCurrentSample();
}

float EasyEsc::currentAmps() const
{
    return readCurrent().amps;
}

float EasyEsc::currentMilliamps() const
{
    return currentAmps() * 1000.0f;
}

uint16_t EasyEsc::currentMilliVolts() const
{
    return readCurrent().milliVolts;
}

void EasyEsc::setCurrentCalibration(float zeroOffsetMv, float mvPerAmp)
{
    _ctrl.setCurrentCalibration(zeroOffsetMv, mvPerAmp);
    _cfg.currentZeroOffsetMv = _ctrl.currentZeroOffsetMv();
    _cfg.currentMvPerAmp = _ctrl.currentMvPerAmp();
}

bool EasyEsc::calibrateCurrentZero(uint16_t samples, uint16_t sampleDelayMs)
{
    if (!hasCurrentMonitor() || samples == 0)
    {
        _lastStatus = Status::InvalidArg;
        return false;
    }

    double sumMv = 0.0;
    for (uint16_t i = 0; i < samples; ++i)
    {
        const CurrentSample sample = readCurrent();
        sumMv += static_cast<double>(sample.milliVolts);
        if (sampleDelayMs > 0)
        {
            delay(sampleDelayMs);
        }
    }

    const float zeroMv = static_cast<float>(sumMv / static_cast<double>(samples));
    setCurrentCalibration(zeroMv, currentMvPerAmp());
    _lastStatus = Status::Ok;
    return true;
}

float EasyEsc::currentZeroOffsetMv() const
{
    return _ctrl.currentZeroOffsetMv();
}

float EasyEsc::currentMvPerAmp() const
{
    return _ctrl.currentMvPerAmp();
}

uint8_t EasyEsc::motorCount() const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.motorCount();
    }
    return requestedMotorCount();
}

gpio_num_t EasyEsc::motorPin(uint8_t motor) const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.motorPin(motor);
    }
    if (motor >= requestedMotorCount())
    {
        return GPIO_NUM_NC;
    }
    return _cfg.motorPins[static_cast<size_t>(motor)];
}

dshot_mode_t EasyEsc::dshotMode() const
{
    if (_ctrl.isInitialized())
    {
        return _ctrl.dshotMode();
    }
    return _cfg.dshotMode;
}

const char *EasyEsc::dshotModeName() const
{
    return _ctrl.dshotModeName();
}

int32_t EasyEsc::lastRmtErrorCode() const
{
    return _ctrl.lastDriverErrorCode();
}

int8_t EasyEsc::lastRmtErrorMotor() const
{
    return _ctrl.lastDriverErrorMotor();
}

gpio_num_t EasyEsc::lastRmtErrorPin() const
{
    return _ctrl.lastDriverErrorPin();
}

MotorTelemetry EasyEsc::motorTelemetry(uint8_t motor) const
{
    return _ctrl.motorTelemetry(motor);
}

Status EasyEsc::lastStatus() const
{
    return _lastStatus;
}

uint8_t EasyEsc::requestedMotorCount() const
{
    return static_cast<uint8_t>(constrain(static_cast<int>(_cfg.motorCount), 1, static_cast<int>(kMaxMotors)));
}

bool EasyEsc::isMotorPinSuitable(gpio_num_t pin)
{
    return isMotorPinSuitableImpl(pin);
}

bool EasyEsc::isCurrentPinSuitable(gpio_num_t pin)
{
    return isCurrentPinSuitableImpl(pin);
}

bool EasyEsc::areConfiguredPinsSuitable() const
{
    const int requested = constrain(static_cast<int>(_cfg.motorCount), 1, static_cast<int>(kMaxMotors));
    for (int i = 0; i < requested; ++i)
    {
        if (!isMotorPinSuitable(_cfg.motorPins[static_cast<size_t>(i)]))
        {
            return false;
        }
    }
    return isCurrentPinSuitable(_cfg.currentPin);
}

EscControllerConfig EasyEsc::makeControllerConfig(uint8_t motorCount) const
{
    EscControllerConfig cfg;
    cfg.motorCount = motorCount;
    cfg.motorPins = _cfg.motorPins;
    cfg.motorDirectionReversed = _cfg.motorDirectionReversed;
    cfg.motorTxBufferSymbols = _cfg.motorTxBufferSymbols;
    cfg.bidirectionalDshot = _cfg.bidirectionalDshot;
    cfg.dshotMode = _cfg.dshotMode;
    cfg.throttleMinRaw = kDshotThrottleMinRaw;
    cfg.throttleMaxRaw = kDshotThrottleMaxRaw;
    cfg.signalTimeoutMs = _cfg.signalTimeoutMs;
    cfg.holdArmOnSignalTimeout = _cfg.holdArmOnSignalTimeout;
    cfg.outputRefreshMs = _cfg.outputRefreshMs;
    cfg.current.enabled = (_cfg.currentPin != GPIO_NUM_NC);
    cfg.current.pin = _cfg.currentPin;
    cfg.current.zeroOffsetMv = _cfg.currentZeroOffsetMv;
    cfg.current.mvPerAmp = _cfg.currentMvPerAmp;
    return cfg;
}

bool EasyEsc::applyStatus(Status s)
{
    _lastStatus = s;
    return s == Status::Ok;
}

EasyEscMotor::EasyEscMotor(
    gpio_num_t motorPin,
    gpio_num_t currentPin,
    dshot_mode_t mode,
    uint32_t signalTimeoutMs,
    uint16_t outputRefreshMs,
    float currentZeroOffsetMv,
    float currentMvPerAmp,
    bool reversed,
    bool bidirectionalDshot,
    uint16_t txBufferSymbols)
    : _esc(1,
           motorPin,
           GPIO_NUM_NC,
           GPIO_NUM_NC,
           GPIO_NUM_NC,
           currentPin,
           mode,
           signalTimeoutMs,
           outputRefreshMs,
           currentZeroOffsetMv,
           currentMvPerAmp,
           reversed,
           false,
           false,
           false,
           bidirectionalDshot,
           txBufferSymbols,
           kDefaultRmtTxBufferSymbols,
           kDefaultRmtTxBufferSymbols,
           kDefaultRmtTxBufferSymbols)
{
}

bool EasyEscMotor::begin()
{
    return _esc.begin();
}

void EasyEscMotor::update()
{
    _esc.update();
}

bool EasyEscMotor::arm()
{
    return _esc.arm();
}

bool EasyEscMotor::disarm()
{
    return _esc.disarm();
}

bool EasyEscMotor::stop()
{
    return _esc.stopAll();
}

bool EasyEscMotor::spinRaw(uint16_t raw)
{
    return _esc.setMotorRaw(0, raw);
}

bool EasyEscMotor::spinPercent(float percent)
{
    return _esc.setMotorPercent(0, percent);
}

bool EasyEscMotor::enterPassthrough()
{
    return _esc.enterPassthrough(0);
}

bool EasyEscMotor::exitPassthrough()
{
    return _esc.exitPassthrough();
}

bool EasyEscMotor::isInPassthrough() const
{
    return _esc.isInPassthrough();
}

uint8_t EasyEscMotor::passthroughRead(uint16_t address, uint8_t *data, uint8_t length)
{
    return _esc.passthroughRead(address, data, length);
}

uint8_t EasyEscMotor::passthroughWrite(uint16_t address, const uint8_t *data, uint8_t length)
{
    return _esc.passthroughWrite(address, data, length);
}

uint8_t EasyEscMotor::passthroughErase(uint16_t address)
{
    return _esc.passthroughErase(address);
}

bool EasyEscMotor::isArmed() const
{
    return _esc.isArmed();
}

bool EasyEscMotor::isInitialized() const
{
    return _esc.isInitialized();
}

void EasyEscMotor::setTimeoutMs(uint32_t timeoutMs)
{
    _esc.setTimeoutMs(timeoutMs);
}

uint32_t EasyEscMotor::timeoutMs() const
{
    return _esc.timeoutMs();
}

void EasyEscMotor::setHoldArmOnSignalTimeout(bool enabled)
{
    _esc.setHoldArmOnSignalTimeout(enabled);
}

bool EasyEscMotor::holdArmOnSignalTimeout() const
{
    return _esc.holdArmOnSignalTimeout();
}

void EasyEscMotor::setRefreshMs(uint16_t refreshMs)
{
    _esc.setRefreshMs(refreshMs);
}

uint16_t EasyEscMotor::refreshMs() const
{
    return _esc.refreshMs();
}

bool EasyEscMotor::hasCurrentMonitor() const
{
    return _esc.hasCurrentMonitor();
}

gpio_num_t EasyEscMotor::currentPin() const
{
    return _esc.currentPin();
}

CurrentSample EasyEscMotor::readCurrent() const
{
    return _esc.readCurrent();
}

float EasyEscMotor::currentAmps() const
{
    return _esc.currentAmps();
}

float EasyEscMotor::currentMilliamps() const
{
    return _esc.currentMilliamps();
}

uint16_t EasyEscMotor::currentMilliVolts() const
{
    return _esc.currentMilliVolts();
}

void EasyEscMotor::setCurrentCalibration(float zeroOffsetMv, float mvPerAmp)
{
    _esc.setCurrentCalibration(zeroOffsetMv, mvPerAmp);
}

bool EasyEscMotor::calibrateCurrentZero(uint16_t samples, uint16_t sampleDelayMs)
{
    return _esc.calibrateCurrentZero(samples, sampleDelayMs);
}

float EasyEscMotor::currentZeroOffsetMv() const
{
    return _esc.currentZeroOffsetMv();
}

float EasyEscMotor::currentMvPerAmp() const
{
    return _esc.currentMvPerAmp();
}

gpio_num_t EasyEscMotor::pin() const
{
    return _esc.motorPin(0);
}

dshot_mode_t EasyEscMotor::dshotMode() const
{
    return _esc.dshotMode();
}

const char *EasyEscMotor::dshotModeName() const
{
    return _esc.dshotModeName();
}

int32_t EasyEscMotor::lastRmtErrorCode() const
{
    return _esc.lastRmtErrorCode();
}

int8_t EasyEscMotor::lastRmtErrorMotor() const
{
    return _esc.lastRmtErrorMotor();
}

gpio_num_t EasyEscMotor::lastRmtErrorPin() const
{
    return _esc.lastRmtErrorPin();
}

MotorTelemetry EasyEscMotor::telemetry() const
{
    return _esc.motorTelemetry(0);
}

Status EasyEscMotor::lastStatus() const
{
    return _esc.lastStatus();
}

bool EasyEscMotor::isMotorPinSuitable() const
{
    return EasyEsc::isMotorPinSuitable(pin());
}

bool EasyEscMotor::isCurrentPinSuitable() const
{
    return EasyEsc::isCurrentPinSuitable(currentPin());
}

bool EasyEscMotor::arePinsSuitable() const
{
    return isMotorPinSuitable() && isCurrentPinSuitable();
}
} // namespace esc
