# Micromouse

An autonomous micromouse designed and built at the Technical University of Munich (TUM) to explore and solve a 6×6 full-size maze and find its center. This project was done as part of the master practical course *Micromouse: Designing an Educational Racing-Robot from Scratch* under supervision of Dr. Alexander Lenz.

This mouse was built by the group of: Jázmin, Paulo, Ágnes, and Kai.

## Table of Contents

- [Micromouse](#micromouse)
  - [Table of Contents](#table-of-contents)
  - [Features](#features)
  - [Repository Structure](#repository-structure)
  - [Hardware Design](#hardware-design)
  - [Software Architecture](#software-architecture)

## Features

- **Hardware**  
  - dsPIC33FJ64MC804 microcontroller  
  - GP2Y0A51 analog distance sensors (front, left, right)  
  - ICM-20948 9-axis IMU for orientation
  - Toshiba TB6612FNG H-Bridge with Faulhaber DC motors  
  - 64×32 OLED display for real-time maze mapping  
  - RN4871 Bluetooth LE module & UART for data streaming  
  - Piezo buzzer (RTTTL melodies)  
  - 7.4 V battery input with 5 V/3.3 V linear regulators  
- **Firmware**  
  - UART & I²C drivers (sync/async)
  - OLED graphics library (SSD1306)
  - Timers & external interrupts with callback registration  
  - Odometry (encoders + gyro + distance sensors)  
  - Positional PD controller with feed-forward & side-wall PI  
  - Trapezoidal velocity profiles for straight & turning motions  
  - Flood-fill navigation with exploration & optimized final run  
- **Scripts**  
  - Python scripts for sensor calibration, feed-forward gain fitting, velocity profiling, and plotting  
  - Data analysis & visualization scripts and CSV data 

## Repository Structure

```
.
├── 3d/         # 3D-printed part designs (Autodesk Fusion) -> TODO: Add files
├── data/       # Raw & processed data logs
│ ├── *.csv
│ └── processed/
├── img/        # Figures/plots from experiments
├── imu_cal/    # IMU calibration scripts & data
├── logo/       # Team and TUM logos
├── pcb/        # Eagle files, design rule files & BOMs
├── scripts/    # Python tools for calibration & plotting
├── src/        # Embedded firmware source & build files
│ ├── adc.c/.h, i2c.c/.h, imu.c/.h, …
│ └── Makefile # Build configuration for MPLAB/xc16
└── README.md
```

## Hardware Design

1. **Schematic & PCB**  
   - Eagle files under `pcb/` (schematics, board, design rules, footprints).  
   - ADC decoupling / low-pass filter, star-shaped power rails, and IMU in center
2. **Sensors & Peripherals**  
   - Distance sensors: analog voltage → distance polynomial fit
   - IMU: offline accel/mag ellipsoid fit + gyro bias calibration on startup.  
   - OLED: SSD1306 over I²C for maze display.  
3. **Mechanical**  
   - Motor & sensor mounts (3D-printed; see `3d/`).  
   - Battery holder below PCB for low center of gravity.  

## Software Architecture

- **Configuration**
  - `src/constants.h`
  - `src/IOconfig.h/.c`
- **Drivers**  
  - `src/dma.c`, `src/clock.c`, `src/adc.c`, `src/uart.c`, `src/i2c.c`, `src/timers.c`, `src/pwm.c`, `src/switches.c`, `src/ssd1306.c`, …
- **Applications**  
  - `src/globalTimers.c` – provides a microsecond-resolution system timer running since power-up.  
  - `src/mazeSolver.c` – implements the flood-fill algorithm for exploration and the optimized run to the maze center.  
  - `src/oled.c` – manages the SSD1306 display’s frame buffer, draws the maze and updates only the regions that change.  
  - `src/rtttl.c` – decodes RTTTL ringtone strings and drives the piezo buzzer to play melodies.  
  - `src/imu.c` – interfaces with the 9-axis IMU over I²C, applies calibration, and reads gyro/accel/mag data.  
  - `src/sensor.s` – reads raw ADC values from the distance sensors and converts them into calibrated distance measurements.  
  - `src/motors.c` – controls the motor driver to set each wheel’s speed as a percentage of maximum.  
  - `src/motorEncoders.c` – reads quadrature encoder counts and uses a PLL to estimate wheel angular velocity.  
- **Control**  
  - `src/mouseController.c` – implements positional PD control with feed-forward terms for precise trajectory tracking  
  - `src/move.c` – generates trapezoidal velocity profiles for both linear and rotational movements  
- **Utilities**  
  - `src/data_visualizer/*.dvws` for creating interactive plots in MPLAB’s Data Visualizer  
- **Unused**
  - `src/odometry.c` – abandoned sensor-fusion odometry module, superseded by integrated controller-based estimation.  