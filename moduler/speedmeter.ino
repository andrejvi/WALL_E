#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4LCD lcd;

float speedMeter() {

  static uint8_t lastDisplayTime;
  static uint16_t countsPermeter = 17; // Dette stemmer ikke, og må måles med målebånd.


  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    int16_t countsGjennomsnitt = ((countsLeft + countsRight) / 2);

    int16_t countsPersekund = countsGjennomsnitt * 10;

    int16_t mps = countsPersekund / countsPermeter;

    lcd.clear();
    lcd.print(countsLeft);
    lcd.gotoXY(0, 1);
    lcd.print(countsRight);
    // lcd.print(mps);

    return mps; // returnerer meter per sekund.
  }
}
