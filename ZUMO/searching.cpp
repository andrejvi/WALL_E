#include <Wire.h>
#include <Zumo32U4.h>
#include "searching.h"

// This might need to be tuned for different lighting conditions,
// surfaces, etc.
#define QTR_THRESHOLD     800  // microseconds

// These might need to be tuned for different motor types.
#define REVERSE_SPEED     200  // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  400  // ms
#define TURN_DURATION     600  // ms

//Counter for hvor mange ganger Walle treffer kanten



void searching(int16_t line_sensor_values[NUM_SENSORS]) {
  //TODO: lage en funksjon som kjører rundt og snur når den treffer en linje:
  //TODO: Hvis den finner en bruskanne, ta den med til avfallstsjon:

  line_sensors.read(line_sensor_values);

  if (line_sensor_values[0] > QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

    kantteller += 1;
  }
  else if (line_sensor_values[NUM_SENSORS - 1] > QTR_THRESHOLD)
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

    kantteller += 1;
  }
  else
  {
    // Otherwise, go straight.
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}
