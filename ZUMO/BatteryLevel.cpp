#include <Wire.h>
#include <Zumo32U4.h>
#include "BatteryLevel.h"
#include "speedmeter.h"

unsigned long batteryLife = 2000; // Total batteri i [mAh]

float batteryLevel(unsigned long avgSpeed){ // Funksjon for batterinivå
  batteryLife -= (5 * avgSpeed); // hastigheten batteriet forbrukes med, basert på fart.

  if (batteryLife >= 0){ // Så lenge batteriet er over 0 returneres batterinivået, og printes til LCD
    return batteryLife;
    }
  else{ // Når batterinivået når 0, returnerer funksjonen 0, og skrivet ut "Empty" på LCD skjermen
    return 0;
    }
  }
