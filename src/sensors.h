#ifndef SENSORS_H
#define	SENSORS_H

typedef enum {
    SENSOR_RIGHT = 0,
    SENSOR_LEFT,
    SENSOR_CENTER
} Sensor_t;

/**
 * @brief Retrieves the distance measurement in voltage from the specified sensor.
 * Takes a sensor enumerator and returns a 16-bit integer representing the measured voltage.
 */
uint16_t getSensorDistanceVoltage(Sensor_t sensor);

/**
 * @brief Retrieves the distance measurement from the specified sensor.
 * Takes a sensor enumerator and returns an 8-bit integer representing the measured distance.
 */
uint8_t getSensorDistance(Sensor_t sensor);

/**
 * @brief Converts a voltage reading to a distance for the specified sensor.
 * Takes a sensor enumerator and a 16-bit voltage, returning an 8-bit integer for the calculated distance in cm.
 * This conversion takes the calibration done for each sensor in advance into account.
 */
uint8_t sensorVoltageToDistance(Sensor_t sensor, uint16_t voltage);

/**
 * @brief Converts a voltage reading to a distance for the specified sensor.
 * Takes a sensor enumerator and a 16-bit voltage, returning an 8-bit integer for the calculated distance  in mm.
 * This conversion takes the calibration done for each sensor in advance into account.
 */
uint8_t sensorVoltageToDistanceInMili(Sensor_t sensor, uint16_t voltage);

#endif	/* SENSORS_H */

