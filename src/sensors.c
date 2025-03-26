#include <stdint.h>
#include "sensors.h"
#include "dma.h"

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

uint16_t getSensorDistanceVoltage(Sensor_t sensor){
    return adcData[sensor];
}

uint16_t getSensorDistance(Sensor_t sensor){
    //return sensorVoltageToDistance(sensor, getSensorDistanceVoltage(sensor));
    return sensorVoltageToDistanceInMili(sensor, getSensorDistanceVoltage(sensor));
}


//#define FAST //Comment to use a more accurate but slower distance fit
uint16_t sensorVoltageToDistance(Sensor_t sensor, uint16_t voltage) {
    SensorParams params = sensorParams[sensor];
   
    float distance = 0.0;
    float v = voltage * 0.001;

    #ifdef FAST
        distance = params.A_f / (v + params.B_f) + params.C_f;
    #else
        distance = params.A / (v + params.B) + params.C / ((v + params.D) * (v + params.D)) + params.E;
    #endif
    
    return (uint16_t)distance;
}

uint16_t sensorVoltageToDistanceInMili(Sensor_t sensor, uint16_t voltage) {
    SensorParams params = sensorParams[sensor];
   
    float distance = 0.0;
    float v = voltage * 0.001;

    #ifdef FAST
        distance = params.A_f / (v + params.B_f) + params.C_f;
    #else
        distance = params.A / (v + params.B) + params.C / ((v + params.D) * (v + params.D)) + params.E;
    #endif
    
    return (uint16_t)(distance * 10);
}