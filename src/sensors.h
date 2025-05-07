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
 * Takes a sensor enumerator and returns an 16-bit integer representing the measured distance.
 */

/* Sensor-only distance (no offset) */
uint16_t getSensorDistance(Sensor_t s);

uint16_t getSensorDistanceMm(Sensor_t s);

uint32_t getSensorDistanceUm(Sensor_t s);

/* Robot distance = sensor distance + offset */
uint32_t getRobotDistanceUm(Sensor_t s);

uint16_t getRobotDistanceMm(Sensor_t s);

void waitForSensorUpdate();

uint32_t averageRobotDistanceUm(Sensor_t s, uint32_t numReadings);

uint32_t medianRobotDistanceUm(Sensor_t s, uint32_t numReadings);

#endif	/* SENSORS_H */

