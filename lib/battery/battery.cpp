#include "battery.h"

const uint8_t battery_pin = 1;
const double R1 = 10.0;
const double R2 = 100.0;
const double battery_k = (R1 + R2) / R1 / 6; // 15k 100k resistor divider plus 6 cells
const uint32_t min_voltage = 3500;       // millivolts
bool battery_empty = false;

void init_battery()
{
    pinMode(battery_pin, INPUT);
    measure_battery();
}

uint32_t measure_battery()
{
    uint32_t state = analogReadMilliVolts(battery_pin) * battery_k;
    if (state < min_voltage)
    {
        battery_empty = true;
    }
    return state;
}
