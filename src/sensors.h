/* 
 * File:   sensors.h
 * Author: kai.helli
 *
 * Created on March 20, 2025, 11:07 AM
 */

#ifndef SENSORS_H
#define	SENSORS_H

typedef enum {
    SENSOR_LEFT = 0,
    SENSOR_CENTER,
    SENSOR_RIGHT
} Sensor_t;


uint16_t getSensorDistanceVoltage(Sensor_t sensor);
uint8_t getSensorDistance(Sensor_t sensor);
uint8_t sensorVoltageToDistance(Sensor_t sensor, uint16_t voltage);

#endif	/* SENSORS_H */

