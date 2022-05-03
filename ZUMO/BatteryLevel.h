#pragma once

#include <Zumo32U4.h>
#include "BatteryLevel.h"

extern float batteryLife;
extern int chargingCount;
extern int batteryHealthPresentage;

int batteryLevel(float counts_no_reset);
uint16_t batteryHealth();
