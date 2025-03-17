# Maze Traversal using Flood Fill
v1.0

## PID Control 
First implementation in demo project
```c
/*
 *         _
 *   /\/\ (_) ___ _ __ ___  _ __ ___   ___  _   _ ___  ___
 *  /    \| |/ __| '__/ _ \| '_ ` _ \ / _ \| | | / __|/ _ \
 * / /\/\ \ | (__| | | (_) | | | | | | (_) | |_| \__ \  __/
 * \/    \/_|\___|_|  \___/|_| |_| |_|\___/ \__,_|___/\___|
 *
 * Example program to showcase the Micromouse API for the demo mouse.
 *
 * File:   main.c
 *
 */

#include "UART.h"
#include "boot.h"
#include "configBits.h"
#include "encoders.h"
#include "globalTimer.h"
#include "leds.h"
#include "motors.h"
#include "sensors.h"
#include "switches.h"
#include "timers.h"

#include <stdio.h>
#include <xc.h>

/* EXAMPLE FUNCTIONS */

/**
 * Routine to toggle LED1 using a timer. This functions signature matches the TimerCallback typedef from the timers.h file.
 * @return 1 to keep the timer running, if 0 is returned the timer will stop
 */
int toggle_led1(void)
{
    LED1 = !LED1;
    return 1; // return 1 to keep the timer running
}

/**
 * Routine to toggle LED2 using the button SW1. This functions signature matches the SwitchCallback typedef from the switches.h file.
 * Returns nothing.
 */
void toggle_led2(void)
{
    LED2 = !LED2;
}

/**
 * Routine to toggle LED3 using the UART RX interrupt. This functions signature matches the UARTCallback typedef from the UART.h file.
 */
void toggle_led3(unsigned int registerContent)
{
    LED3 = !LED3;
}

/**
 * Routine to toggle LED4 using the UART TX interrupt. This functions signature matches the UARTCallback typedef from the UART.h file.
 */
void toggle_led4(unsigned int registerContent)
{
    LED4 = !LED4;
}

/**
 * Routine to stream sensor readings over UART. This can be used to visualize the sensor readings in MPLAB Data Visualizer.
 *
 * Workspace file "sensor_test.dvws" can be found in this directory to setup the correct variable streamer.
 *
 * See MPLAB X Visualizer documentation for the protocol: MPLABÂ® Data Visualizer User's Guide (DS50003001) - 5.2 Stream Format
 */
int stream_sensor_data(void)
{
    LED1 = !LED1;
    UART_TXint8(FRAME_MARKER); // start of visualizer frame
    UART_TXint8(sensors_readFront());
    UART_TXint8(sensors_readLeft());
    UART_TXint8(sensors_readRight());
    UART_TXUInt(sensors_readFrontRaw());
    UART_TXUInt(sensors_readLeftRaw());
    UART_TXUInt(sensors_readRightRaw());
    UART_TXLong(encoders_getAngleLeftDegrees());
    UART_TXLong(encoders_getAngleRightDegrees());
    UART_TXFloat(encoders_getAngleLeftRadians());
    UART_TXFloat(encoders_getAngleRightRadians());
    UART_TXFloat(encoders_getRPMLeft());
    UART_TXFloat(encoders_getRPMRight());
    UART_TXFloat(encoders_getRadPerSecLeft());
    UART_TXFloat(encoders_getRadPerSecRight());
    UART_TXLong(encoders_getTicksLeft());
    UART_TXLong(encoders_getTicksRight());
    UART_TXint8(~FRAME_MARKER); // end of visualizer frame (one's complement of start marker)
    return 1;                   // return 1 to keep the timer running
}

/* Prints the current time since boot in ms to UART, formatted in ASCII */
int transmit_time(void)
{
    char timestring[20];
    sprintf(timestring, "Time: %lu\n", globalTimer_getTimeInUS() / 1000); // devide by 1000 to convert to ms
    UART_TXString(timestring);
    return 1;
}

/**
 * Routine to toggle the motors using the button SW1. This functions signature matches the SwitchCallback typedef from the switches.h file.
 */
void toggle_motors(void)
{
    static int running = 0;
    if (running)
    {
        motors_setMotorLeft(0);
        motors_setMotorRight(0);
        running = 0;
    }
    else
    {
        motors_setMotorLeft(0.3);
        motors_setMotorRight(0.3);
        running = 1;
    }
}

struct PID{
    float K_p;
    float K_i;
    float K_d;
    float last_error;
    //output
    float w_l;
    float w_r;
    float w_f;
};

//PID struct
struct PID pid_state;

//state 
static int started = 0;

void init_PID(struct PID* pid) {
    pid->K_p = 0.0007;
    pid->K_i = 0;
    pid->K_d = 0.0001;
    pid->last_error = 0;
    pid->w_l = 0;
    pid->w_r = 0;
    pid->w_f = 0;
}

void update_PID(struct PID* pid, uint8_t left, uint8_t right, uint8_t front) {
    //TODO set w_f according to front distance
    float error = (float)(left - right);
    pid->w_l = pid->w_f - pid->K_p * error - pid->K_d * (error - pid->last_error);
    pid->w_r = pid->w_f + pid->K_p * error + pid->K_d * (error - pid->last_error);
    if(pid->w_l > 1) pid->w_l = 1.0;
    if(pid->w_r > 1) pid->w_r = 1.0;
    if(pid->w_l < -1) pid->w_l = -1.0;
    if(pid->w_r < -1) pid->w_r = -1.0;
    pid->last_error = error;
}

void toggle_control() {
    
    //initialize PID
    init_PID(&pid_state);
    
    if (started)
    {
        motors_setMotorLeft(0);
        motors_setMotorRight(0);
        started = 0;
    }
    else
    {   
        pid_state.w_f = 0.35;
        started = 1;
    } 
}

/* Prints the current time since boot in ms to UART, formatted in ASCII */
int update_control(void)
{   
    if (started) {
        uint8_t left = sensors_readLeft();
        uint8_t right = sensors_readRight();
        uint8_t front = sensors_readFront();
        update_PID(&pid_state, left, right, front);
        motors_setMotorLeft(pid_state.w_l);
        motors_setMotorRight(pid_state.w_r);
    }
    return 1;
}

/* MAIN FUNCTION */

int main() {
    
    LED1 = LEDOFF;
    LED2 = LEDOFF;
    LED3 = LEDOFF;
    LED4 = LEDOFF;

    // initializes the peripherals, must be called first
    boot_setup();

    // initializes the UART1 module with a baud rate of 115200
    UART_setup(115200);

    // Example timer usage
    timers_initTimerInMS(1, 419);
    timers_registerTimerCallback(1, &update_control);
    timers_startTimer(1);

    // test switches
    switches_registerSW1Callback(&toggle_control);

    // infinite loop
    while (1) {}
    return 0;
}

int main1()
{   
    //PID struct
    struct PID pid;
    
    LED1 = LEDOFF;
    LED2 = LEDOFF;
    LED3 = LEDOFF;
    LED4 = LEDOFF;

    // initializes the peripherals, must be called first
    boot_setup();

    // initializes the UART1 module with a baud rate of 115200
    UART_setup(115200);

    //initialize PID
    init_PID(&pid);
    
    // Example timer usage
    timers_initTimerInMS(1, 419);
    timers_registerTimerCallback(1, &transmit_time);
    timers_startTimer(1);

    // test switches
    switches_registerSW1Callback(&toggle_motors);

    // test UART callbacks
    UART_registerRX1Callback(&toggle_led3);
    UART_registerTX1Callback(&toggle_led4);

    // infinite loop
    while (1) {}
    return 0;
}
```


