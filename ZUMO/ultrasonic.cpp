#include <Wire.h>
#include <Zumo32U4.h>

#define echoPin 17 // attach pin D2 Arduino to pin Echo of HC-SR04
#define trigPin 13 //attach pin D3 Arduino to pin Trig of HC-SR04

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement
unsigned long period_1 = 2;
unsigned long period_2 = 10;
unsigned long timenow = 0;

int ultrasonic() {
  // Clears the trigPin condition
  while (micros() > timenow + period_1) {
    timenow += micros();
    digitalWrite(trigPin, LOW);
  }

  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  while (micros() > timenow + period_2) {
    timenow += micros();
    digitalWrite(trigPin, HIGH);
  }

  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  return distance;
}
