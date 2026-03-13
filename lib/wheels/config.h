#pragma once

#include <TMC2209.h>

#define M_R_SENSE 0.11f

#define M_DRIVE_EN_PIN 15      // Enable is common to all four drive motors
#define M_DRIVE_RX 18          // For setting TMC2209 parameters
#define M_DRIVE_TX 17          // For setting TMC2209 parameters
#define M_DRIVE_SERIAL Serial1 // For setting TMC2209 parameters
#define M_DRIVE_STEPS_PER_TURN 200
#define M_DRIVE_MICROSTEP 32 // default motor microstep

#define M1_DRIVER_ADDRESS TMC2209::SERIAL_ADDRESS_0
#define M1_DIR_PIN 4
#define M1_STP_PIN 5

#define M2_DRIVER_ADDRESS TMC2209::SERIAL_ADDRESS_1
#define M2_DIR_PIN 6
#define M2_STP_PIN 7

#define M3_DRIVER_ADDRESS TMC2209::SERIAL_ADDRESS_2
#define M3_DIR_PIN 8
#define M3_STP_PIN 3

#define M4_DRIVER_ADDRESS TMC2209::SERIAL_ADDRESS_3
#define M4_DIR_PIN 9
#define M4_STP_PIN 10

// Motion
#define MAX_SPEED_WHEELS 500 // mm/s
#define MAX_ACCEL_WHEELS 500 // mm/s^2

#define ODOM_CORRECTION_TRANSLATION 1.045 // (100, 100) 1.041
#define ODOM_CORRECTION_ROTATION 1.012    //
