#include <Wire.h>
#include <Zumo32U4.h>
#include "searching.h"

// This might need to be tuned for different lighting conditions,
// surfaces, etc.
#define QTR_THRESHOLD     1000  // microseconds

// These might need to be tuned for different motor types.
#define REVERSE_SPEED     200  // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  200  // ms
#define TURN_DURATION     300  // ms



void searching() {
  //TODO: lage en funksjon som kjører rundt og snur når den treffer en linje:
  //TODO: Hvis den finner en bruskanne, ta den med til avfallstsjon:

  lineSensors.read(line_sensor_values);

  if (line_sensor_values[0] < QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (line_sensor_values[NUM_SENSORS - 1] < QTR_THRESHOLD)
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // Otherwise, go straight.
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}
