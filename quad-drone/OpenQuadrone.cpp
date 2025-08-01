#include "OpenQuadrone.h"
#include <math.h>
#include <TimerMicros.h>

#define GYRO_SENS  65.5f    // LSB sensitivity ±500°/s
#define ACCE_SENS  4096.0f  // LSB sensitivity ±8g
TimerMicros kalmantimer(4000);

OpenQuadrone::OpenQuadrone(bool customIMU, uint8_t sdaPin, uint8_t sclPin, uint8_t address)
  : _customIMU(customIMU),
    _sda(sdaPin), _scl(sclPin), _addr(address),
    _offsetRoll(0), _offsetPitch(0), _offsetYaw(0),
    _rawRoll(0), _rawPitch(0), _rawYaw(0),
    _accX(0), _accY(0), _accZ(0),
    _angleYaw(0), _prevMicrosYaw(0),
    _velZ(0), _altitude(0), _prevMicrosAlt(0),
    _kalmanAngleRoll(0), _kalmanUncertaintyAngleRoll(4),
    _kalmanAnglePitch(0), _kalmanUncertaintyAnglePitch(5)
{
}

void OpenQuadrone::begin() {
    if (!_customIMU) {
        Wire.setClock(400000);
        if (_sda == 0 && _scl == 0) Wire.begin();
        else Wire.begin(_sda, _scl);
        writeReg(0x6B, 0x00);  // Wake-up
        delay(250);

        writeReg(0x1A, 0x05);  // DLPF a 10 Hz
        writeReg(0x1B, 0x08);  // ±500°/s
        writeReg(0x1C, 0x10);  // ±8g

        calibrateIMU();
        // inicializa temporização para Yaw e Altitude
        _prevMicrosYaw = micros();
        _prevMicrosAlt = micros();
    }
}

void OpenQuadrone::calibrateIMU(uint16_t samples, uint16_t delayMs) {
    double sumR=0, sumP=0, sumY=0;
    for (uint16_t i = 0; i < samples; ++i) {
        readRaw();
        sumR += _rawRoll;
        sumP += _rawPitch;
        sumY += _rawYaw;
        delay(delayMs);
    }
    _offsetRoll  = sumR / samples;
    _offsetPitch = sumP / samples;
    _offsetYaw   = sumY / samples;
}

void OpenQuadrone::getRates(float& rollR, float& pitchR, float& yawR) {
    readRaw();
    rollR  = _rawRoll  - _offsetRoll;
    pitchR = _rawPitch - _offsetPitch;
    yawR   = _rawYaw   - _offsetYaw;
}

void OpenQuadrone::getAcc(float& x, float& y, float& z) {
    readRaw();
    x = _accX;
    y = _accY;
    z = _accZ;
}

void OpenQuadrone::getAnglesAcc(float& roll, float& pitch) {
    readRaw();
    // cálculo de roll e pitch via acelerômetro
    float r = atan(_accY / sqrt(_accX * _accX + _accZ * _accZ));
    float p = -atan(_accX / sqrt(_accY * _accY + _accZ * _accZ));
    roll  = r * RAD_TO_DEG;
    pitch = p * RAD_TO_DEG;
}

void OpenQuadrone::getAngles(float& rollA, float& pitchA, float&yawA){
    kalmanFilter();
    rollA = _kalmanAngleRoll;
    pitchA = _kalmanAnglePitch;
    // integração de yaw
    unsigned long now = micros();
    float dt = (now - _prevMicrosYaw) / 1e6f;
    _prevMicrosYaw = now;

    float dummyR, dummyP;
    getRates(dummyR, dummyP, _rawYaw);
    _angleYaw += _rawYaw * dt;
    // Normalização em [0,360)
    _angleYaw = fmod(_angleYaw, 360.0f);
    if (_angleYaw < 0.0f) _angleYaw += 360.0f;

    yawA = _angleYaw;
}

void OpenQuadrone::getAltitude(float& altitude) {
    readRaw();
    unsigned long now = micros();
    float dt = (now - _prevMicrosAlt) / 1e6f;
    _prevMicrosAlt = now;

    // _accZ está em g (após calibração de offset de aceleração)
    float aZ = _accZ * GRAVITY;        // m/s²

    // ZUPT: se acelerômetro próximo de 0 (sem movimento)
    if (fabs(aZ) < 0.1f) {
        _velZ = 0;
    } else {
        _velZ += aZ * dt;               // integração para velocidade
    }

    _altitude += _velZ * dt;            // integração para posição
    altitude = _altitude;
}

void OpenQuadrone::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

void OpenQuadrone::readRaw() {
    // Giroscópio
    Wire.beginTransmission(_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)6);
    int16_t gx = (Wire.read()<<8) | Wire.read();
    int16_t gy = (Wire.read()<<8) | Wire.read();
    int16_t gz = (Wire.read()<<8) | Wire.read();
    _rawRoll  = gx / GYRO_SENS;
    _rawPitch = gy / GYRO_SENS;
    _rawYaw   = gz / GYRO_SENS;

    // Acelerômetro
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)6);
    int16_t ax = (Wire.read()<<8) | Wire.read();
    int16_t ay = (Wire.read()<<8) | Wire.read();
    int16_t az = (Wire.read()<<8) | Wire.read();
    _accX = ax / ACCE_SENS - 0.03;
    _accY = ay / ACCE_SENS;
    _accZ = az / ACCE_SENS - 0.11;
}

void OpenQuadrone::kamanl1D(float angle, float gyro, float &state, float &uncertainty) {
    static constexpr float dt             = 0.004f;
    static constexpr float processNoise   = 0.00512f;
    static constexpr float measurementNoise = 9.0f;

    state       += gyro * dt;
    uncertainty += processNoise;

    float gain = uncertainty / (uncertainty + measurementNoise);
    state      += gain * (angle - state);
    uncertainty *= (1.0f - gain);
}

void OpenQuadrone::kalmanFilter() {
    if (kalmantimer.pronto()) {
        float angleR, angleP, angleY;
        float rateR, rateP, rateY;
        getAnglesAcc(angleR, angleP);
        getRates(rateR, rateP, rateY);
        kamanl1D(angleR, rateR, _kalmanAngleRoll, _kalmanUncertaintyAngleRoll);
        kamanl1D(angleP, rateP, _kalmanAnglePitch, _kalmanUncertaintyAnglePitch);
    }
}