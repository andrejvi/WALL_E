#pragma once

#include <Zumo32U4.h>
#include "ultrasonic.h"

// Variabler
#define echoPin 3 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 2 //attach pin D3 Arduino to pin Trig of HC-SR04

long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
unsigned long period_1 = 2;
unsigned long period_2 = 10;
unsigned long timenow = 0;

// Funksjoner
int ultrasonic()
