#pragma once

#include <TMC2209.h>
#include "FastAccelStepper.h"
#include "config.h"

struct wheelsState {
    double w1; // rad/s
    double w2; // rad/s
    double w3; // rad/s
    double w4; // rad/s
    double p1; // rad
    double p2; // rad
    double p3; // rad
    double p4; // rad
};

class Wheels
{
public:
    Wheels();
    void disable_motors();
    void enable_motors();
    void set_speed(double w1, double w2, double w3, double w4);
    wheelsState get_current_state();

private:
    TMC2209 *_M1_driver;
    TMC2209 *_M2_driver;
    TMC2209 *_M3_driver;
    TMC2209 *_M4_driver;

    FastAccelStepperEngine _stepper_engine = FastAccelStepperEngine();
    FastAccelStepper *_M1_stepper = NULL;
    FastAccelStepper *_M2_stepper = NULL;
    FastAccelStepper *_M3_stepper = NULL;
    FastAccelStepper *_M4_stepper = NULL;

    // Constant for: (Steps per Rev / 2PI) * 1000ms * Microsteps (rad/s to milliHz)
    const double _K = (M_DRIVE_STEPS_PER_TURN / (2.0 * M_PI)) * 1000.0 * M_DRIVE_MICROSTEP;
    const int32_t _ACCEL = 5 * M_DRIVE_STEPS_PER_TURN * M_DRIVE_MICROSTEP;
    const int32_t _min_speed_millihz = 11*1000;  // Should at least go at 10hz, or else stop

    gpio_num_t _en_pin;

    bool _enabled;

    void set_RMS(uint16_t current);
    void set_microstep(uint16_t ms);
    bool is_running();
};