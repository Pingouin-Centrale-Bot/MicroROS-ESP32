#pragma once

#include <TMC2209.h>
#include "FastAccelStepper.h"
#include "config.h"

class Wheels
{
public:
    Wheels();
    void disable_motors();
    void enable_motors();
    void set_speed(double w1, double w2, double w3, double w4);

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

    // Constant for: (Steps per Rev / 2PI) * 1000ms * Microsteps
    const double _K = (M_DRIVE_STEPS_PER_TURN / (2.0 * M_PI)) * 1000.0 * M_DRIVE_MICROSTEP;
    const int32_t _ACCEL = 5 * M_DRIVE_STEPS_PER_TURN * M_DRIVE_MICROSTEP;

    gpio_num_t _en_pin;

    bool _enabled;

    void set_RMS(uint16_t current);
    void set_microstep(uint16_t ms);
    bool is_running();
};