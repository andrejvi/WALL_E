#pragma once

#include <Zumo32U4.h>
#include "ultrasonic.h"

// Variabler
extern const unsigned long DISTANCE_SENSOR_TIMEOUT_US;

// Funksjoner
float distance_reading();
