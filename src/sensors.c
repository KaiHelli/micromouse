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
    {15.2959, 0.3116, -1.9764, 0.3114, -4.0133, 18.9997, 0.6148, -5.1902},  // Fit Parameters for SENSOR_RIGHT
    {14.7042, 0.2326, -1.9796, 0.2331, -3.7681, 21.4925, 0.6542, -5.9672},  // Fit Parameters for SENSOR_LEFT
    {17.9144, 0.3052, -2.8008, 0.3050, -5.2426, 27.5355, 0.8280, -8.0530}   // Fit Parameters for SENSOR_CENTER
};

uint16_t getSensorDistanceVoltage(Sensor_t sensor){
    return adcData[sensor];
}

uint8_t getSensorDistance(Sensor_t sensor){
    return sensorVoltageToDistance(sensor, getSensorDistanceVoltage(sensor));
}


#define FAST //Comment to use a more accurate but slower distance fit
uint8_t sensorVoltageToDistance(Sensor_t sensor, uint16_t voltage) {
    SensorParams params = sensorParams[sensor];
   
    float distance = 0.0;
    float v = voltage * 0.001;

    #ifdef FAST
        distance = params.A_f / (v + params.B_f) + params.C_f;
    #else
        distance = params.A / (v + params.B) + params.C / ((v + params.D) * (v + params.D)) + params.E;
    #endif
    
    return (uint8_t)distance;
}

uint8_t sensorVoltageToDistanceInMili(Sensor_t sensor, uint16_t voltage) {
    SensorParams params = sensorParams[sensor];
   
    float distance = 0.0;
    float v = voltage * 0.001;

    #ifdef FAST
        distance = params.A_f / (v + params.B_f) + params.C_f;
    #else
        distance = params.A / (v + params.B) + params.C / ((v + params.D) * (v + params.D)) + params.E;
    #endif
    
    return (uint8_t)(distance * 10);
}