/* Wall-E
    Kode for Zumo32U4 - Gruppeprosjekt vår 2022.

    Prosedyre: ZUMO kjører rundt innenfor en innhegning av svart teip og rydder
               halvlitersbokser til det er tomt.

    Hardware:  ZUMO32u4 beltebilrobot
               HCSR04 ultrasonisk distansesensor
               Kretskort for seriellkommunikasjon med TTGO-ESP32 over RX/TX
*/


//Biblioteker:
#include <Wire.h> //Init
#include <Zumo32U4.h> //Zumo bibliotek
#include <Zumo32U4Buzzer.h>
#include "BatteryLevel.h" //Batterifunksjoner
#include "speedmeter.h" //Speedmeter
#include "searching.h" //søkefunksjon
#include "ZumoState.h"
#include "Packet.h"
#include "ultrasonic.h"
#include "Bank.h"

#define State ZumoState

// Konfigurerer pakke
#define PACKAGE_START_BYTE 0b00111100   // "<"
#define PACKAGE_STOP_BYTE  0b00111110   // ">"
const uint8_t PACKAGE_SIZE = sizeof(Package);

// Debug-printing over seriell
#define DEBUG_PRINT_TO_SERIAL true

// Skru av eller på lyden
#define SILENT true

// Konstanter
#define NUM_SENSORS 5
const uint8_t TRIG_PIN = 14;
const uint8_t ECHO_PIN = 17;
const int8_t LCD_UPDATE_DELAY_MS = 10;
const unsigned long CALIBRATION_TIME_MS = 2000;
const unsigned long DISTANCE_SENSOR_TIMEOUT_US = 3000; // Gir oss ca 39 cm range
const uint16_t MS_BETWEEN_AUTOMATIC_PACKAGE_TRANSMISSIONS = 150;
const int8_t PID_DEFAULT_P = 2;
const int8_t PID_DEFAULT_I = 0;
const int8_t PID_DEFAULT_D = 6;
const int16_t LOW_BATTERY = 200;
const float LOWER_DISTANCE = 2.0;
const float MAX_DISTANCE = 20.0;
int ladePris = 3; // koster 3kr/mAh
int startSaldo = 10000; // Starter med 10tusen kroner

const int16_t QTR_THRESHOLD = 800;

// Farter
const int16_t MAX_SPEED = 400;
const int16_t NORMAL_SPEED = 200;
const int16_t BACKING_SPEED = 190;
const int16_t APPROACH_SPEED = 150;
const int16_t TURN_SPEED = 100;
const int16_t SCAN_SPEED = 120;
const int16_t LINE_FOLLOW_SPEED = 200;
const int16_t RETURN_SPEED = 200;



struct PidRegulator {
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

    return (_P + _I + _D) / 5;
  }
};


enum Direction { LEFT_TO_RIGHT, RIGHT_TO_LEFT };


// Instansiering av globale objekter
Zumo32U4LineSensors line_sensors;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
State state = State::RESET;
State state_prev;
PidRegulator pid;
Package received_package;
Package local_package;


// Globale variabler
bool linesensors_calibrated_since_last_powerup = false;
bool state_has_changed;
bool require_package_transmission;
bool moving_away_from_wall = false;
bool right_sensor_hit = false;
bool left_sensor_hit = false;
unsigned long time_in_state;
unsigned long time_0;
unsigned long time_since_transmission;
unsigned long time_moving_away;
int16_t line_sensor_values[NUM_SENSORS];
int16_t line_position;
float ultrasonic_distance_reading;
int32_t max_speed = MAX_SPEED;
int32_t left_speed = 0;
int32_t right_speed = 0;
int32_t lastDisplayTime;
float countsLeft;
float countsRight;
float counts_no_reset;
uint8_t serial_buffer[sizeof(local_package)];
int kantteller;
float batteryLife = 2000;
Direction last_turn_direction;
int chargingCount;
int mAhCharged = 0;
uint16_t saldo;
int batteryHealthPresentage = 100;


bool receive_serial_package(uint8_t serial_buffer[PACKAGE_SIZE]) {
  // Returnerer enten "false" om vi ikke har fått inn pakke, eller
  // "true" dersom en ny pakke er skrevet over på "received_package".

  bool receive_in_progress = false;
  byte index = 0;
  uint8_t received_byte;

  while (Serial1.available() > 0) {
    received_byte = Serial1.read();

    if (received_byte == PACKAGE_START_BYTE) {
      receive_in_progress = true;
    }

    if (receive_in_progress) {
      if (received_byte != PACKAGE_STOP_BYTE) {
        serial_buffer[index] = received_byte;
      }

      if ((received_byte == PACKAGE_STOP_BYTE) || (index == PACKAGE_SIZE)) {
        // Vi har mottatt pakke, kopierer nå over i "received_package"
        memcpy(&received_package, serial_buffer, PACKAGE_SIZE);

        return true;
      }

      if (index > PACKAGE_SIZE) {
        // Ingen stopbyte, noe har gått galt
        return false;
      }

      index ++;
    }
  }
  // Ingen pakke har kommet inn
  return false;
}


bool detect_box(int16_t max_distance) {
  if ((LOWER_DISTANCE < ultrasonic_distance_reading) && (ultrasonic_distance_reading < max_distance)) {
    return true;
  } else {
    return false;
  }
}


void setup() {
  line_sensors.initFiveSensors();

  Serial1.begin(115200);
  if (DEBUG_PRINT_TO_SERIAL) Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}


void loop() {
  ///////////////////////////////////////////////////////////////////////////////
  //                                                                           //
  //     Operasjoner som kjøres hver runde før tilstandsmaskinen oppdateres    //
  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////
  require_package_transmission = false;
  state_prev = state;

  // Ser om det kommer en ny pakke fra ESP
  if (receive_serial_package(serial_buffer)) {

    // Leser over de variablene fra pakka vi ønsker å bruke i zumoen
    if (received_package.update_zumo_state) state = received_package.zumo_state;
    if (received_package.update_Kp)         pid.Kp = received_package.Kp;
    if (received_package.update_Ki)         pid.Ki = received_package.Ki;
    if (received_package.update_Kd)         pid.Kd = received_package.Kd;
    if (received_package.update_Kd)         pid.Kd = received_package.Kd;

    // Vi ønsker å sende et svar for å bekrefte til den andre ESPen
    require_package_transmission = true;
  }


  state_prev = state;

  // Oppdaterer sensormålinger
  line_position = line_sensors.readLine(line_sensor_values);
  line_sensors.readCalibrated(line_sensor_values);
  ultrasonic_distance_reading = distance_reading();
  float avg_speed = abs(speedmeter(countsLeft, countsRight));


  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    //Henter antall counts fra encoder, i løpet av 0.1 sekunder.
    countsLeft = encoders.getCountsAndResetLeft();
    countsRight = encoders.getCountsAndResetRight();
  }

  //Teller antall counts siden programstart
  counts_no_reset = (encoders.getCountsLeft() + encoders.getCountsRight()) / 2;

  // Vi bruker batteri
  batteryLevel(counts_no_reset);

  //WALLE bruker spenn
  saldo = Saldo(mAhCharged, ladePris, startSaldo);


  ///////////////////////////////////////////////////////////////////////////////
  //                                                                           //
  //                       Oppdaterer Tilstandsmaskinen                        //
  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////
  switch (state) {
    case State::RESET: {
        // Wall-E restarter

        // Resetter alle verdier
        kantteller = 0;

        // Kjører kalibrering om det ikke allerede er gjort
        if (!linesensors_calibrated_since_last_powerup) {
          time_0 = millis();
          state = State::CALIBRATE_LINESENSORS;
        } else {
          state = State::WAIT_FOR_START_SIGNAL;
        }
      } break;

    case State::CALIBRATE_LINESENSORS: {
        // Wall-E kjører kalibreringsprosedyren
        if (!SILENT) buzzer.play("!T240 L8 agafaea dac+adaea fa<aa<bac#a dac#adaea f4");
        if (state_has_changed) {
          time_0 = millis();
        }

        // Kjør rundt i ring i 4 sekunder
        motors.setSpeeds(-NORMAL_SPEED, NORMAL_SPEED);
        line_sensors.calibrate();

        if ((millis() - time_0) > CALIBRATION_TIME_MS) {
          linesensors_calibrated_since_last_powerup = true;
          state = State::WAIT_FOR_START_SIGNAL;
        }
      } break;

    case State::WAIT_FOR_START_SIGNAL: {
        // Wall-E venter på signal om å starte

        // Mens vi debugger går vi til SEARCHING etter en stund
        if (millis() - time_in_state > 2000) {
          // GÅR DIREKTE TIL SEARCHING FOR BOX
          state = State::SEARCHING_FOR_BOX;
        }
      } break;

    case State::SEARCHING_FOR_BOX: {
        // Wall-E kjører rundt inne i byen og ser etter bokser.

        motors.setSpeeds(NORMAL_SPEED, NORMAL_SPEED);

        if (kantteller >= 5) {
          // Etter å ha truffet veggen fem ganger, prøv å scanne
          state = State::SCANNING_FOR_BOX;
        }


        // Ser om vi har truffet kanten
        if ((line_sensor_values[0] > QTR_THRESHOLD) || (line_sensor_values[4] > QTR_THRESHOLD)) {

          // Vi traff kanten, sjekk hvilken sensor som traff
          if (line_sensor_values[0] > QTR_THRESHOLD) last_turn_direction = Direction::RIGHT_TO_LEFT;
          if (line_sensor_values[4] > QTR_THRESHOLD) last_turn_direction = Direction::LEFT_TO_RIGHT;

          kantteller += 1;

          state = State::BACKING_FROM_BORDER;
        }

        float avg_speed = speedmeter(countsLeft, countsRight);

        if (batteryLife < LOW_BATTERY) {
          // Kjør til ladestasjon
          state = State::RETURN_TO_STATION;
        }
        else if (batteryLife == 0) {
          //dødt batteri
          state = State::STOPPED;
        }

        if (detect_box(20)) {
          time_0 = millis();
          state = State::FOUND_BOX;
        }
      } break;

    case State::BACKING_FROM_BORDER: {
        // Wall-E rygger vekk fra kanten

        motors.setSpeeds(-BACKING_SPEED, -BACKING_SPEED);

        if (millis() - time_in_state > 400) {
          state = State::SMALL_TURN;
        }
      } break;

    case State::SMALL_TURN: {
        // Wall-E snur seg litt for å unngå å kjøre samme vei

        if (last_turn_direction = Direction::RIGHT_TO_LEFT) {
          motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        }
        if (last_turn_direction = Direction::LEFT_TO_RIGHT) {
          motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        }

        if (millis() - time_in_state > 1500) {
          state = State::SEARCHING_FOR_BOX;
        }
      } break;

    case State::SCANNING_FOR_BOX: {
        // Wall-E scanner etter bokser ved å snu rundt i ring

        kantteller = 0;

        motors.setSpeeds(-SCAN_SPEED, SCAN_SPEED);

        if (millis() - time_in_state > 3000) {
          state = State::SEARCHING_FOR_BOX;
        }

        if (detect_box(35)) {
          time_0 = millis();
          last_turn_direction = Direction::LEFT_TO_RIGHT;
          state = State::TURNING_TO_BOX;
        }
      } break;

    case State::FOUND_BOX: {
        // Wall-E fant en boks
        if (!SILENT) buzzer.play("!T240 L8 cdeacdea");

        if (millis() - time_in_state > 1000) {
          state = State::MOVING_TO_BOX;
          time_0 = millis();
        }
      } break;

    case State::TURNING_TO_BOX: {
        // Wall-E snur seg den siste lille biten for å sentrere på boksen

        switch (last_turn_direction) {
          case Direction::RIGHT_TO_LEFT:
            motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
            break;
          case Direction::LEFT_TO_RIGHT:
            motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
            break;
        }

        // Vi snur oss i 100 ms etter først å ha detektert boksen. Dette er for å passe
        // på at Wall-E peker rett mot boksen og ikke kjører på den sidelengs så den kanter
        if (millis() - time_in_state > 100) {
          if (detect_box(35)) {
            state = State::FOUND_BOX;
          } else {
            state = State::LOST_TRACK_OF_BOX;
          }
        }
      } break;

    case State::LOST_TRACK_OF_BOX: {
        // Wall-E kan ikke lenger se boksen

        // Gå til SEARCHING_FOR_BOX etter en stund
        if (millis() - time_in_state > 1000) {
          kantteller = 0;
          state = State::SEARCHING_FOR_BOX;
        }

        // TODO: spill av en trist lyd på buzzeren
      } break;

    case State::MOVING_TO_BOX: {
        // Wall-E beveger seg sakte mot boksen

        motors.setSpeeds(APPROACH_SPEED, APPROACH_SPEED);

        // Pass på at vi ikke forlater banen
        if (line_sensor_values[2] > 900) {
          state = State::RETURN_TO_CITY;
        }

        // Gjør kontinuerlige målinger med ultralydsensoren mens vi nærmer oss
        if (ultrasonic_distance_reading < 5) {
          state = State::GRABBING_BOX;
        }
      } break;

    case State::GRABBING_BOX: {
        // Wall-E griper tak i boksen.

        state = State::MOVE_TO_BORDER;
      } break;

    case State::MOVE_TO_BORDER: {
        // Wall-E tar med seg boksen ut til linja

        motors.setSpeeds(APPROACH_SPEED, APPROACH_SPEED);

        if (line_sensor_values[2] > QTR_THRESHOLD) {
          // Vi fant kanten
          state = State::RETURN_TO_STATION;
        }
      } break;

    case State::RETURN_TO_STATION: {
        // Wall-E følger linja tilbake til ladestasjonen

        max_speed = LINE_FOLLOW_SPEED;

        // Følger linja hele veien til ladestasjonen
        int16_t error = line_position - 2000;
        int32_t speed_difference = pid.GetOutput(error);

        left_speed = constrain((max_speed + speed_difference), -max_speed / 3, max_speed);
        right_speed = constrain((max_speed - speed_difference), -max_speed / 3, max_speed);
        motors.setSpeeds(left_speed, right_speed);

        // Ser etter den svarte stopplinja
        if (line_sensor_values[0] > QTR_THRESHOLD && line_sensor_values[4] > QTR_THRESHOLD) {
          // Vi fant den svarte stopplinja (ladestasjonen)

          // Luker ut falske positive med å legge inn en delay
          if (millis() - time_in_state > 2000) {
            state = State::REFUELING;
          }
        }
      } break;

    case State::REFUELING: {
        // Wall-E er på ladestasjonen og fyller opp batteriene
        if (!SILENT) buzzer.play("!T240 L4 cdef");
        if (batteryLife < 2000) {
          batteryLife += 1;
          mAhCharged += 1;
        }

        // Dra tilbake når batteriet er fullt
        if (batteryLife >= 2000) {
          chargingCount += 1;
          state = State::RETURN_TO_CITY;
        }
      } break;

    case State::STOPPED: {
        // Wall-E har stoppet opp

        motors.setSpeeds(0, 0);
        // TODO: Spill av en alarmlyd eller liknende på buzzeren
      } break;

    case State::FOLLOW_LINE: {
        // Wall-E følger linje. Denne tilstanden er ment til debugging av linjefølgeren.
        max_speed = LINE_FOLLOW_SPEED;
        int16_t error = line_position - 2000;
        int32_t speed_difference = pid.GetOutput(error);

        left_speed = constrain((max_speed + speed_difference), -max_speed / 3, max_speed);
        right_speed = constrain((max_speed - speed_difference), -max_speed / 3, max_speed);
        motors.setSpeeds(left_speed, right_speed);
      } break;

    case State::RETURN_TO_CITY: {
        // Wall-E rygger tilbake til byen

        if (millis() - time_in_state < 2500) {
          // Rygger kjapt tilbake til byen
          motors.setSpeeds(-RETURN_SPEED, -RETURN_SPEED);
        } else {
          // Etter å ha rygga i 2500 ms snur vi litt rundt
          motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        }

        if (millis() - time_in_state > 2800) {
          state = State::SEARCHING_FOR_BOX;
        }
      } break;
  }

  ///////////////////////////////////////////////////////////////////////////////
  //                                                                           //
  //    Operasjoner som kjøres hver runde etter tilstandsmaskinen oppdateres   //
  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////
  state_has_changed = (state_prev != state);


  if (state_has_changed) {
    if (DEBUG_PRINT_TO_SERIAL) {
      Serial.print("state has changed to ");
      Serial.println(zumo_state_to_str(state));
    }

    time_in_state = millis();
    require_package_transmission = true;

    // Hver gang vi endrer tilstand skrur vi motorene av
    motors.setSpeeds(0, 0);
  }

  if ((millis() - time_since_transmission) > MS_BETWEEN_AUTOMATIC_PACKAGE_TRANSMISSIONS) {
    // Sender pakker etter regelmessige tidsintervaller
    require_package_transmission = true;
  }

  if (require_package_transmission) {
    // Legger ny info over i pakke og sender til ESP over Serial1
    local_package.stop_byte = PACKAGE_STOP_BYTE;
    local_package.zumo_state = state;
    local_package.Kp = pid.Kp;
    local_package.Ki = pid.Ki;
    local_package.Kd = pid.Kd;
    local_package.battery_level = batteryLife;
    local_package.speed = avg_speed;
    local_package.ultrasonic_distance_reading = ultrasonic_distance_reading;
    local_package.battery_real = batteryHealth();
    local_package.bank_balance = Saldo(mAhCharged, ladePris, startSaldo);
    local_package.start_byte = PACKAGE_START_BYTE;

    Serial1.write((uint8_t*)&local_package, PACKAGE_SIZE);

    if (DEBUG_PRINT_TO_SERIAL) Serial.write((uint8_t*)&local_package, PACKAGE_SIZE);

    require_package_transmission = false;
    time_since_transmission = millis();
  }
}
