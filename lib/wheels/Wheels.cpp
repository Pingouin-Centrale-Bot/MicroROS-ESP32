#include "Wheels.h"
#include "config.h"

Wheels::Wheels()
{
    _M1_driver = new TMC2209();
    _M2_driver = new TMC2209();
    _M3_driver = new TMC2209();
    _M4_driver = new TMC2209();

    _M1_driver->setup(M_DRIVE_SERIAL, 115200, M1_DRIVER_ADDRESS, M_DRIVE_RX, M_DRIVE_TX);
    _M1_driver->setReplyDelay(4);
    _M2_driver->setup(M_DRIVE_SERIAL, 115200, M2_DRIVER_ADDRESS, M_DRIVE_RX, M_DRIVE_TX);
    _M2_driver->setReplyDelay(4);
    _M3_driver->setup(M_DRIVE_SERIAL, 115200, M3_DRIVER_ADDRESS, M_DRIVE_RX, M_DRIVE_TX);
    _M3_driver->setReplyDelay(4);
    _M4_driver->setup(M_DRIVE_SERIAL, 115200, M4_DRIVER_ADDRESS, M_DRIVE_RX, M_DRIVE_TX);
    _M4_driver->setReplyDelay(4);

    _M1_driver->enableAutomaticCurrentScaling();
    _M2_driver->enableAutomaticCurrentScaling();
    _M3_driver->enableAutomaticCurrentScaling();
    _M4_driver->enableAutomaticCurrentScaling();

    _M1_driver->enableAutomaticGradientAdaptation();
    _M2_driver->enableAutomaticGradientAdaptation();
    _M3_driver->enableAutomaticGradientAdaptation();
    _M4_driver->enableAutomaticGradientAdaptation();

    set_RMS(M_DRIVE_CURRENT_MA);
    set_microstep(M_DRIVE_MICROSTEP);

    _M1_driver->enable();
    _M2_driver->enable();
    _M3_driver->enable();
    _M4_driver->enable();

    _stepper_engine.init();
    _M1_stepper = _stepper_engine.stepperConnectToPin(M1_STP_PIN);
    _M1_stepper->setDirectionPin(M1_DIR_PIN);
    _M1_stepper->setAcceleration(_ACCEL);
    _M1_stepper->setForwardPlanningTimeInMs(10);

    _M2_stepper = _stepper_engine.stepperConnectToPin(M2_STP_PIN);
    _M2_stepper->setDirectionPin(M2_DIR_PIN);
    _M2_stepper->setAcceleration(_ACCEL);
    _M2_stepper->setForwardPlanningTimeInMs(10);

    _M3_stepper = _stepper_engine.stepperConnectToPin(M3_STP_PIN);
    _M3_stepper->setDirectionPin(M3_DIR_PIN);
    _M3_stepper->setAcceleration(_ACCEL);
    _M3_stepper->setForwardPlanningTimeInMs(10);

    _M4_stepper = _stepper_engine.stepperConnectToPin(M4_STP_PIN);
    _M4_stepper->setDirectionPin(M4_DIR_PIN);
    _M4_stepper->setAcceleration(_ACCEL);
    _M4_stepper->setForwardPlanningTimeInMs(10);

    _en_pin = (gpio_num_t)M_DRIVE_EN_PIN;
    gpio_reset_pin(_en_pin);
    gpio_set_direction(_en_pin, GPIO_MODE_OUTPUT);

    disable_motors();
    log_i("Motors initialized");
}

void Wheels::disable_motors()
{
    log_i("Disabling motors");
    gpio_set_level(_en_pin, 1);
    _enabled = true;
}

void Wheels::enable_motors()
{
    log_i("Enabling motors");
    gpio_set_level(_en_pin, 0);
    _enabled = false;
}

void Wheels::set_speed(double w1, double w2, double w3, double w4)
{
    double speeds[4] = {w1, w2, w3, w4};
    uint32_t speeds_milliHz[4];
    // 1. Storage for processed data
    FastAccelStepper *motors[] = {_M1_stepper, _M2_stepper, _M3_stepper, _M4_stepper};

    // 2. Phase One: Update all frequencies first
    for (int i = 0; i < 4; i++)
    {
        speeds_milliHz[i] = abs(speeds[i]) * _K;
        if (speeds_milliHz[i] >= _min_speed_millihz)
            motors[i]->setSpeedInMilliHz(speeds_milliHz[i]);
    }

    // 3. Phase Two: Update all directions/states immediately after
    for (int i = 0; i < 4; i++)
    {
        if (speeds_milliHz[i] < _min_speed_millihz)
        {
            motors[i]->stopMove();
        }
        else if (speeds[i] > 0)
        {
            motors[i]->runForward();
        }
        else
        {
            motors[i]->runBackward();
        }
    }

    log_v("Running motors at [%d, %d, %d, %d]milliHz", speeds_milliHz[0], speeds_milliHz[1], speeds_milliHz[2], speeds_milliHz[3]);
}

wheelsState Wheels::get_current_state()
{
    wheelsState wheels_state = wheelsState();

    wheels_state.w1 = _M1_stepper->getCurrentSpeedInMilliHz(false) / _K;
    wheels_state.w2 = _M2_stepper->getCurrentSpeedInMilliHz(false) / _K;
    wheels_state.w3 = _M3_stepper->getCurrentSpeedInMilliHz(false) / _K;
    wheels_state.w4 = _M4_stepper->getCurrentSpeedInMilliHz(false) / _K;

    // Not precise for ESP32, but won't fix because will soon be ported to rp2350
    wheels_state.p1 = _M1_stepper->getCurrentPosition() / _K * 1000.0;
    wheels_state.p2 = _M2_stepper->getCurrentPosition() / _K * 1000.0;
    wheels_state.p3 = _M3_stepper->getCurrentPosition() / _K * 1000.0;
    wheels_state.p4 = _M4_stepper->getCurrentPosition() / _K * 1000.0;

    return wheels_state;
}

bool Wheels::is_running()
{
    return _enabled || _M1_stepper->isRunning() || _M2_stepper->isRunning() || _M3_stepper->isRunning() || _M4_stepper->isRunning();
}

void Wheels::set_RMS(uint16_t current)
{
    _M1_driver->setRMSCurrent(current, M_R_SENSE);
    _M2_driver->setRMSCurrent(current, M_R_SENSE);
    _M3_driver->setRMSCurrent(current, M_R_SENSE);
    _M4_driver->setRMSCurrent(current, M_R_SENSE);
}

void Wheels::set_microstep(uint16_t ms)
{
    _M1_driver->setMicrostepsPerStep(ms);
    _M2_driver->setMicrostepsPerStep(ms);
    _M3_driver->setMicrostepsPerStep(ms);
    _M4_driver->setMicrostepsPerStep(ms);
}
