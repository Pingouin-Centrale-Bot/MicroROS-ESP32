#pragma once

#include <Arduino.h>

extern bool battery_empty;

void init_battery();
uint32_t measure_battery();