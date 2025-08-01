// OpenQuadrone.cpp
#include "OpenQuadrone.h"
#include <math.h>
#include <TimerMicros.h>

#define GYRO_SENS  65.5f    // LSB/°/s
#define ACCE_SENS  4096.0f  // LSB/g

OpenQuadrone::OpenQuadrone(bool customIMU,
                           uint8_t sdaPin,
                           uint8_t sclPin,
                           uint8_t address)
  : _customIMU(customIMU),
    _sda(sdaPin), _scl(sclPin), _addr(address),
    _offsetRoll(0), _offsetPitch(0), _offsetYaw(0),
    _ratesLPF(ACC_LPF_ALPHA),
    _attitude(1.0f, 0.0f, 0.0f, 0.0f),
    _prevMicrosAtt(0),
    _velZ(0), _altitude(0), _prevMicrosAlt(0)
{}

void OpenQuadrone::begin() {
    Wire.setClock(400000);
    if (_sda==0 && _scl==0) Wire.begin();
    else                     Wire.begin(_sda, _scl);

    writeReg(0x6B, 0x00);
    delay(250);
    writeReg(0x1A, 0x05);
    writeReg(0x1B, 0x08);
    writeReg(0x1C, 0x10);

    calibrateIMU();
    _prevMicrosAtt = micros();
    _prevMicrosAlt = micros();
}

void OpenQuadrone::calibrateIMU(uint16_t samples, uint16_t delayMs) {
    long sum[3] = {0, 0, 0};
    for (uint16_t i = 0; i < samples; ++i) {
        readRaw();
        delay(delayMs);
        sum[0] += _rawGyro[0];
        sum[1] += _rawGyro[1];
        sum[2] += _rawGyro[2];
    }
    _offsetRoll  = sum[0] / (float)samples;
    _offsetPitch = sum[1] / (float)samples;
    _offsetYaw   = sum[2] / (float)samples;
}

void OpenQuadrone::getAngles(float& rollA,
                             float& pitchA,
                             float& yawA) {
    readRaw();
    unsigned long now = micros();
    float dt = (now - _prevMicrosAtt) / 1e6f;
    _prevMicrosAtt = now;

    estimateAttitude(dt);
    auto e = _attitude.toEulerZYX();
    // toEulerZYX returns Vector(z=yaw, y=pitch, x=roll)
    rollA  = e.x;
    pitchA = e.y;
    yawA   = e.z;
}

void OpenQuadrone::estimateAttitude(float dt) {
    Vector rates(
        (_rawGyro[0] - _offsetRoll)  / GYRO_SENS,
        (_rawGyro[1] - _offsetPitch) / GYRO_SENS,
        (_rawGyro[2] - _offsetYaw)   / GYRO_SENS
    );
    rates = _ratesLPF.update(rates);

    Quaternion dq = Quaternion::fromAngularRates(rates * dt);
    _attitude = _attitude * dq;
    _attitude.normalize();

    Vector acc(
        _rawAcc[0] / ACCE_SENS,
        _rawAcc[1] / ACCE_SENS,
        _rawAcc[2] / ACCE_SENS
    );
    if (fabs(acc.norm() - GRAVITY) < GRAVITY * 0.1f) {
        Vector up = _attitude.rotate(Vector(0, 0, GRAVITY));
        Vector corr = Vector::angularRatesBetweenVectors(acc, up) * dt * WEIGHT_ACC;
        Quaternion qc = Quaternion::fromAngularRates(corr);
        _attitude = _attitude * qc;
        _attitude.normalize();
    }
}

void OpenQuadrone::getAltitude(float& altitude) {
    readRaw();
    unsigned long now = micros();
    float dt = (now - _prevMicrosAlt) / 1e6f;
    _prevMicrosAlt = now;

    Vector accB(
        _rawAcc[0] / ACCE_SENS,
        _rawAcc[1] / ACCE_SENS,
        _rawAcc[2] / ACCE_SENS
    );
    Vector accW = _attitude.rotate(accB);
    float aZ = accW.z - GRAVITY;

    if (fabs(aZ) < 0.1f) _velZ = 0;
    else                 _velZ += aZ * dt;

    _altitude += _velZ * dt;
    altitude = _altitude;
}

void OpenQuadrone::readRaw() {
    Wire.beginTransmission(_addr); Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(_addr, 6);
    _rawGyro[0] = ((int16_t)Wire.read() << 8) | Wire.read();
    _rawGyro[1] = ((int16_t)Wire.read() << 8) | Wire.read();
    _rawGyro[2] = ((int16_t)Wire.read() << 8) | Wire.read();
    _rawGyro[0] /= GYRO_SENS;
    _rawGyro[1] /= GYRO_SENS;
    _rawGyro[2] /= GYRO_SENS;

    Wire.beginTransmission(_addr); Wire.write(0x3B); Wire.endTransmission(false);
    Wire.requestFrom(_addr, 6);
    _rawAcc[0] = ((int16_t)Wire.read() << 8) | Wire.read();
    _rawAcc[1] = ((int16_t)Wire.read() << 8) | Wire.read();
    _rawAcc[2] = ((int16_t)Wire.read() << 8) | Wire.read();
}

void OpenQuadrone::writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}