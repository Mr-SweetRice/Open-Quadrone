#ifndef OPENQUADRONE_H
#define OPENQUADRONE_H

#include <Arduino.h>
#include <Wire.h>

class OpenQuadrone {
public:
    // customIMU = false → usa MPU6050; SDA/SCL default aos pinos 0 e 1
    OpenQuadrone(bool customIMU = false,
                 uint8_t sdaPin   = 0,
                 uint8_t sclPin   = 0,
                 uint8_t address  = 0x68);

    void begin();
    void calibrateIMU(uint16_t samples = 1000, uint16_t delayMs = 1);
    void getRates(float& rollR, float& pitchR, float& yawR);
    void getAcc(float& x, float& y, float& z);
    void getAngles(float& rollA, float& pitchA, float& yawA);
    void getAltitude(float& altitude);
    void kalmanFilter();
    void readRaw();

private:
    bool      _customIMU;
    const uint8_t _sda, _scl, _addr;
    float     _offsetRoll, _offsetPitch, _offsetYaw;
    float     _rawRoll, _rawPitch, _rawYaw;
    float     _accX, _accY, _accZ;
    // estados para Yaw integrado
    float     _angleYaw;
    unsigned long _prevMicrosYaw;
    // estados para Altitude integrado
    float     _velZ;
    float     _altitude;
    unsigned long _prevMicrosAlt;

    float     _kalmanAngleRoll, _kalmanUncertaintyAngleRoll;
    float     _kalmanAnglePitch, _kalmanUncertaintyAnglePitch;
    void getAnglesAcc(float& roll, float& pich);
    void writeReg(uint8_t reg, uint8_t val);
    void kamanl1D(float angle, float gyro, float &state, float &uncertainty);
    static constexpr float GRAVITY = 9.81f; // m/s^2
};

#endif  // OPENQUADRONE_H