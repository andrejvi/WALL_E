#include <Wire.h>
#include <Zumo32U4.h>
#include "BatteryLevel.h"
#include "speedmeter.h"

//float batteryLife = 2000; // Total batteri i [mAh]

int batteryLevel(float counts_no_reset){ // Funksjon for batterinivå
  batteryLife -= (0.001 * counts_no_reset); // hastigheten batteriet forbrukes med, basert på fart.

  if (batteryLife >= 0){ // Så lenge batteriet er over 0 returneres batterinivået, og printes til LCD
    return round(batteryLife);
    }
  else{ // Når batterinivået når 0, returnerer funksjonen 0, og skrivet ut "Empty" på LCD skjermen
    return 0;
    }
  }

int batteryHealth(float chargingCount, float counts_no_reset){
    // Batterihelse basert på avstand kjørt og antall ladesykluser
    batteryHealthPresentage = 100 - (chargingCount * (counts_no_reset * 0.001));
    return batteryHealthPresentage;
  }
