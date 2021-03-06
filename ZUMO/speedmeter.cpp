#include <Wire.h>
#include <Zumo32U4.h>
#include "speedmeter.h"

// Funskjon som returnerer fart i cm/s.
float speedmeter(float countsLeft, float countsRight) {
  float rev = 909.7F; //Counts per rev
  float cm_rev = 11.624F; //Cm per rev
  static uint8_t lastDisplayTime;

  // Måler fart hvert 0.1 sekund
  if ((millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();
/*
    //Henter antall counts fra encoder, i løpet av 0.1 sekunder.
    float countsLeft = encoders.getCountsAndResetLeft();
    float countsRight = encoders.getCountsAndResetRight();
*/

    // Regner antall revs på 0.1 sekund.
    float revLeft = countsLeft / rev;
    float revRight = countsRight / rev;

    // Regner om til cm/s
    float cm_s_Left = revLeft * cm_rev * 10;
    float cm_s_Right = revRight * cm_rev * 10;

    // Regner gjennomsnittet til Left- og Righthastighet
    float cm_s_avg = (cm_s_Left + cm_s_Right)/2;
    
    // Returnerer average hastighet.
    return cm_s_avg;
  }
}
