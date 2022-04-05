#pragma once

#include <Zumo32U4.h>
#include "searching.h"

// Globale objekter
extern Zumo32U4Motors motors;
extern Zumo32U4LineSensors lineSensors;

// Variabler
#define NUM_SENSORS 5
extern int16_t line_sensor_values[NUM_SENSORS];


// Funksjoner
void searching();
