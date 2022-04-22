/* Wall-E
    Kode for Zumo32U4 - Gruppeprosjekt vår 2022.
*/

//Biblioteker:
#include <Wire.h> //Init
#include "EEPROM.h" //EEPROM: et minne der variabler er lagret selv når Zumo er skrudd av.
#include <Zumo32U4.h> //Zumo bibliotek
#include "BatteryLevel.h" //Batterifunksjoner
#include "speedmeter.h" //Speedmeter
#include "searching.h" //søkefunksjon
#include "ZumoState.h"
#include "Packet.h"
#include "ultrasonic.h"

#define State ZumoState

// Konfigurerer pakke
#define PACKAGE_START_BYTE 0b00111100   // "<"
#define PACKAGE_STOP_BYTE  0b00111110   // ">"

// Konstanter
const uint8_t PACKAGE_SIZE = sizeof(Package);
const uint8_t CONNECTIONS_PER_NODE = 8;
const uint8_t TOTAL_NODES = 8;
#define NUM_SENSORS 3
const int8_t LCD_UPDATE_DELAY_MS = 10;
const unsigned long CALIBRATION_TIME_MS = 4000;
const int8_t PID_DEFAULT_P = 3;
const int8_t PID_DEFAULT_I = 0;
const int8_t PID_DEFAULT_D = 6;
const int16_t MAX_SPEED = 270;
const int16_t LOW_BATTERY = 200;



struct PID {
  // PID-regulator. Forsterkningskonstanter Kp, Ki og Kd er
  //    gitt i tideler.

  int8_t Kp = PID_DEFAULT_P;
  int8_t Ki = PID_DEFAULT_I;
  int8_t Kd = PID_DEFAULT_D;
  int16_t last_error = 0;

  int32_t GetOutput(int16_t new_error) {
    int32_t _P = Kp * new_error;
    int32_t _I = Ki * (new_error + last_error) / 2;
    int32_t _D = Kd * (new_error - last_error);
    last_error = new_error;

    return (_P + _I + _D) / 10;
  }
};



// Instansiering av globale objekter
Zumo32U4LineSensors line_sensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA button_a;
Zumo32U4ButtonB button_b;
Zumo32U4ButtonC button_c;
Zumo32U4Encoders encoders;
State state = State::RESET;
State state_prev;
PID pid;
Package local_package;


// Globale variabler
bool linesensors_calibrated_since_last_powerup = false;
bool state_has_changed;
bool require_package_transmission;
uint16_t update_counter;
unsigned long time_in_state;
unsigned long time_0;
int16_t line_sensor_values[NUM_SENSORS];
int16_t line_position;
int32_t max_speed = MAX_SPEED;
int32_t left_speed = 0;
int32_t right_speed = 0;
int32_t lastDisplayTime;
float countsLeft;
float countsRight;
float counts_no_reset;
byte serial_buffer[sizeof(local_package)];



bool receive_serial_package(byte serial_buffer[PACKAGE_SIZE]) {
  // Returnerer enten "false" om vi ikke har fått inn pakke, eller
  // "true" dersom en ny pakke er skrevet over på "serial_buffer".

  static bool receive_in_progress = false;
  static byte index = 0;
  byte received_byte;

  while (Serial1.available() > 0) {
    received_byte = Serial1.read();

    if (received_byte == PACKAGE_START_BYTE) {
      receive_in_progress = true;
    }

    if (receive_in_progress) {
      if (received_byte != PACKAGE_STOP_BYTE) {
        serial_buffer[index] = received_byte;
      }

      if (received_byte == PACKAGE_STOP_BYTE) {
        receive_in_progress = false;
        index = 0;

        // Vi har mottatt pakke, kopierer nå over i "local_package"
        memcpy(&local_package, serial_buffer, PACKAGE_SIZE);
        return true;
      }

      index ++;
    }
  }
  return false;
}


void setup() {
  Serial1.begin(115200);
  line_sensors.initThreeSensors();
  //proxSensors.initThreeSensors();
  update_counter = 0;
  Serial.begin(9600);
}


void loop() {
  ///////////////////////////////////////////////////////////////////////////////
  //                                                                           //
  //     Operasjoner som kjøres hver runde før tilstandsmaskinen oppdateres    //
  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////
  state_prev = state;
  require_package_transmission = false;


  if (receive_serial_package(serial_buffer)) {
    // Vi har mottatt ny "local_package"

    require_package_transmission = true;

    // Leser over de variablene fra pakka vi ønsker å bruke i zumoen
    state = local_package.zumo_state;
    pid.Kp = local_package.Kp;
    pid.Ki = local_package.Ki;
    pid.Kd = local_package.Kd;
  }


  line_position = line_sensors.readLine(line_sensor_values);


  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    //Henter antall counts fra encoder, i løpet av 0.1 sekunder.
    countsLeft = encoders.getCountsAndResetLeft();
    countsRight = encoders.getCountsAndResetRight();
  }

  //Teller antall counts siden programstart
  counts_no_reset = (encoders.getCountsLeft() + encoders.getCountsRight()) / 2;


  ///////////////////////////////////////////////////////////////////////////////
  //                                                                           //
  //                       Oppdaterer Tilstandsmaskinen                        //
  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////
  switch (state) {
    case State::RESET: {
        // Wall-E commit sudoku

        // Kjører kalibrering om det ikke allerede er gjort
        if (!linesensors_calibrated_since_last_powerup) {
          time_0 = millis();
          state = State::CALIBRATE_LINESENSORS;
        }
      } break;

    case State::CALIBRATE_LINESENSORS: {
        // Wall-E kjører kalibreringsprosedyren

        // Kjør rundt i ring i 4 sekunder
        left_speed = -100;
        right_speed = 100;
        line_sensors.calibrate();

        if ((millis() - time_0) > CALIBRATION_TIME_MS) {
          linesensors_calibrated_since_last_powerup = true;
          state = State::WAIT_FOR_START_SIGNAL;
        }

        motors.setSpeeds(left_speed, right_speed);
      } break;

    case State::WAIT_FOR_START_SIGNAL: {
        // Wall-E venter på signal om å starte

        Serial.println(ultrasonic());

        // Vent på at noen trykker på knapp A
        if (button_a.getSingleDebouncedPress()) {
          state = State::SEARCHING_FOR_BOX;
        }


        left_speed = 0;
        right_speed = 0;

        // TODO: aksepter startsignal fra ESP32
      } break;

    case State::SEARCHING_FOR_BOX: {
        // Wall-E kjører rundt inne i byen og ser etter bokser.

        //funksjon som kjører rundt innenfor en border
        searching(line_sensor_values[NUM_SENSORS]);

        float avg_speed = speedmeter(countsLeft, countsRight);

        if (batteryLevel(counts_no_reset) < LOW_BATTERY) {
          // Kjør til ladestasjon
          state = State::RETURN_TO_STATION;
        }
        else if (batteryLevel(counts_no_reset) == 0) {
          //dødt batteri
          state = State::STOPPED;
        }


      } break;

    case State::FOUND_BOX: {
        // Wall-E fant en boks

        // TODO: spill av en glad lyd på buzzeren

        // TODO: lagre hvilken sektor vi så boksen i, og snu dit i TURNING_TO_BOX
      } break;

    case State::TURNING_TO_BOX: {
        // Wall-E snur seg mot boksen

        // TODO: her kan vi bruke ultralydsensoren (posisjonert rett frem) til
        //       å ta målinger mens vi snur oss sakte rundt. Når vi tror vi
        //       peker rett mot boksen, gå til MOVING_TO_BOX
        //       Om vi bruker for lang tid uten å se noen boks, gå til LOST_TRACK_OF_BOX.
      } break;

    case State::LOST_TRACK_OF_BOX: {
        // Wall-E kan ikke lenger se boksen

        // TODO: spill av en trist lyd på buzzeren

        // TODO: gå til SEARCHING_FOR_BOX etter en stund
      } break;

    case State::MOVING_TO_BOX: {
        // Wall-E beveger seg sakte mot boksen

        // TODO: gjør kontinuerlige målinger med ultralydsensoren mens vi nærmer oss

        // TODO: gå til GRABBING_BOX når vi er nærme nok
      } break;

    case State::GRABBING_BOX: {
        // Wall-E griper tak i boksen.

        // TODO: når gripeprosedyren er ferdig, gå til MOVE_TO_BORDER
        // TODO: eventuelt, om vi tror vi mista boksen, gå til LOST_TRACK_OF_BOX
      } break;

    case State::MOVE_TO_BORDER: {
        // Wall-E tar med seg boksen ut til linja

        // TODO: når gripeprosedyren er ferdig, gå til MOVE_TO_BORDER
      } break;

    case State::RETURN_TO_STATION: {
        // Wall-E følger linja tilbake til ladestasjonen

        // TODO: følg linja, slik som i FOLLOW_LINE
        // TODO: i motsetning til som i FOLLOW_LINE  må vi stoppe når vi kommer
        //       frem til ladestasjonen.
      } break;

    case State::REFUELING: {
        // Wall-E er på ladestasjonen og fyller opp batteriene
        // TODO: start opp igjen når batteriene er fulle
      } break;

    case State::STOPPED: {
        // Wall-E står stille
        // TODO: start opp igjen på grunnlag av visse kriterier
      } break;

    case State::FOLLOW_LINE: {
        // Wall-E følger linje. Denne tilstanden er ment til debugging av linjefølgeren.
        int16_t error = line_position - 2000;
        int32_t speed_difference = pid.GetOutput(error);

        left_speed = constrain((max_speed + speed_difference), -max_speed / 3, max_speed);
        right_speed = constrain((max_speed - speed_difference), -max_speed / 3, max_speed);
        motors.setSpeeds(left_speed, right_speed);
      } break;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //                                                                           //
  //    Operasjoner som kjøres hver runde etter tilstandsmaskinen oppdateres   //
  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////
  state_has_changed = (state_prev != state);


  if (state_has_changed) {
    time_in_state = millis();
    require_package_transmission = true;

    motors.setSpeeds(0, 0);
  }

  if (require_package_transmission) {
    local_package.stop_byte = PACKAGE_STOP_BYTE;
    local_package.zumo_state = state;
    local_package.Kp = pid.Kp;
    local_package.Ki = pid.Ki;
    local_package.Kd = pid.Kd;
    local_package.start_byte = PACKAGE_START_BYTE;
    
    Serial1.write((byte*)&local_package, PACKAGE_SIZE);
    require_package_transmission = false;
  }


  // Plusser på "1" til "update_counter", med mindre den er MAX_INT, da går den til 0
  update_counter = (update_counter == 65535) ? 0 : update_counter + 1;
}
