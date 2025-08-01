// OpenQuadrone.h
#ifndef OPENQUADRONE_H
#define OPENQUADRONE_H

#include <Arduino.h>
#include <Wire.h>
#include "quaternion.h"
#include "vector.h"
#include "lpf.h"

/**
 * OpenQuadrone: Classe de fusão de sensores com filtro
 * complementar em quaternions para atitude e dupla
 * integração de aceleração Z para altitude.
 */
class OpenQuadrone {
public:
    OpenQuadrone(bool customIMU = false,
                 uint8_t sdaPin   = 0,
                 uint8_t sclPin   = 0,
                 uint8_t address  = 0x68);

    void begin();
    void calibrateIMU(uint16_t samples = 1000, uint16_t delayMs = 1);
    void getAngles(float& rollA, float& pitchA, float& yawA);
    void getAltitude(float& altitude);

private:
    bool      _customIMU;
    const uint8_t _sda, _scl, _addr;
    float     _offsetRoll, _offsetPitch, _offsetYaw;

    int16_t   _rawGyro[3];
    int16_t   _rawAcc[3];

    Quaternion            _attitude;
    LowPassFilter<Vector> _ratesLPF;
    static constexpr float WEIGHT_ACC      = 0.5f;
    unsigned long         _prevMicrosAtt;

    float     _velZ;
    float     _altitude;
    unsigned long _prevMicrosAlt;
    static constexpr float GRAVITY         = 9.81f;
    static constexpr float ACC_LPF_ALPHA   = 0.2f;

    void readRaw();
    void writeReg(uint8_t reg, uint8_t val);
    void estimateAttitude(float dt);
};

#endif // OPENQUADRONE_H