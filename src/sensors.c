#include <stdint.h>
#include "sensors.h"
#include "dma.h"

typedef enum {
    UNIT_MILI,   // × 10  ? returns uint16_t
    UNIT_MICRO,  // × 10 000 ? returns uint32_t
} DistanceUnit;

typedef struct {
    float A;
    float B;
    float C;
    float D;
    float E;
    float A_f;
    float B_f;
    float C_f;
} SensorParams;

SensorParams sensorParams[] = {
    {13.7655, 0.2689, -1.7061, 0.2687, -3.2377, 16.9291, 0.5622, -4.1538},  // Fit Parameters for SENSOR_RIGHT
    {13.5351, 0.2085, -1.7247, 0.2088, -3.1044, 18.5562, 0.5829, -4.5765},  // Fit Parameters for SENSOR_LEFT
    {15.6223, 0.2619, -2.2394, 0.2617, -4.0475, 22.2279, 0.7086, -5.8410}   // Fit Parameters for SENSOR_CENTER
};

static const uint32_t sensorOffsetUm[] = {
    /* SENSOR_RIGHT  */  33300,   
    /* SENSOR_LEFT   */  33300,  
    /* SENSOR_CENTER */  54100
};


/* raw ADC reading, 0..4095 for 0..Vref */
uint16_t getSensorDistanceVoltage(Sensor_t s) {
    return adcData[s];
}

/* computeDistance -> centimeters */
static inline float _computeDistanceCm(Sensor_t s, uint16_t voltage) {
    SensorParams p = sensorParams[s];
    float v = voltage * 0.001f;  // scale voltage into volts
#ifdef FAST
    return p.A_f / (v + p.B_f) + p.C_f;
#else
    return p.A  / (v + p.B)
         + p.C  / ((v + p.D) * (v + p.D))
         + p.E;
#endif
}

/* generic converter: returns integer mm or ?m */
static inline uint32_t sensorVoltageToDistance(
    Sensor_t      s,
    uint16_t      voltage,
    DistanceUnit  unit
) {
    float dist_cm = _computeDistanceCm(s, voltage);
    switch (unit) {
      case UNIT_MILI:  // cm -> mm
        return (uint32_t)(dist_cm * 10.0f);
      case UNIT_MICRO: // cm -> ?m
        return (uint32_t)(dist_cm * 10000.0f);
      default:
        return 0u;
    }
}

/* Sensor?only distance (no offset) */
uint16_t getSensorDistance(Sensor_t s) {
    return getSensorDistanceMm(s);
}

uint16_t getSensorDistanceMm(Sensor_t s) {
    return (uint16_t)sensorVoltageToDistance(s,
             getSensorDistanceVoltage(s),
             UNIT_MILI);
}
uint32_t getSensorDistanceUm(Sensor_t s) {
    return sensorVoltageToDistance(s,
             getSensorDistanceVoltage(s),
             UNIT_MICRO);
}

/* Robot distance = sensor distance + offset */
/* ?? in ?m (precise) ?? */
uint32_t getRobotDistanceUm(Sensor_t s) {
    return getSensorDistanceUm(s) + sensorOffsetUm[s];
}
/* ?? in mm (rounded from ?m) ?? */
uint16_t getRobotDistanceMm(Sensor_t s) {
    uint32_t total_um = getRobotDistanceUm(s);
    // round to nearest mm: (um + 500) / 1000
    return (uint16_t)((total_um + 500u) / 1000u);
}