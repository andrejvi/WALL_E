#include <Wire.h>
#include <Zumo32U4.h>

#define echoPin 17 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 14 //attach pin D3 Arduino to pin Trig of HC-SR04


long duration; // variable for the duration of sound wave travel
uint8_t distance; // variable for the distance measurement
unsigned long timenow = 0;


  float distance_reading() {
  // Midlertidig, måtte bare ha en som funka for å se om zumoen kunne drive HCSR04
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH, 3000);
  return (duration * .0343) / 2;
}
