/* Wall-E
 *  Kode for Zumo32U4 - Gruppeprosjekt vår 2022.
 */

//Biblioteker:
#include <Wire.h> //Init
#include "EEPROM.h" //EEPROM: et minne der variabler er lagret selv når Zumo er skrudd av.
#include <Zumo32U4.h> //Zumo bibliotek
#include "BatteryLevel.h" //Batterifunksjoner
#include "speedmeter.h" //Speedmeter
#include "searching.h" //søkefunksjon

// Konstanter
const uint8_t CONNECTIONS_PER_NODE = 8;
const uint8_t TOTAL_NODES = 8;
#define NUM_SENSORS 5
const int8_t LCD_UPDATE_DELAY_MS = 10;
const unsigned long CALIBRATION_TIME_MS = 4000;
const int8_t PID_DEFAULT_P = 3;
const int8_t PID_DEFAULT_I = 0;
const int8_t PID_DEFAULT_D = 6;
const int16_t MAX_SPEED = 270;
const int16_t LOW_BATTERY = 200;


struct BranchNode {
  // Datastruktur som beskriver én forgreining i stien
  uint8_t id;
  uint8_t connections[CONNECTIONS_PER_NODE];
  // TODO: hva er avstanden til nabonodene?
  //     - må finne en smart måte å lagre retning
  // TODO: har denne forgreininga en sensornode?
  // TODO: når besøkte bilen denne noden sist?
};



enum State : uint8_t {
  // Wall-E vil alltid være i én av disse tilstandene
  RESET = 0,
  CALIBRATE_LINESENSORS,
  WAIT_FOR_START_SIGNAL,
  MOVING,
  BRANCH_FOUND,
  MAP_BRANCHPOINT,
  RETURN_TO_STATION,
  BRAKING,
  STOPPED,
  REFUELING,
  SPIRALLING,
  PID_TUNE
};



struct PID {
  // PID-regulator. Forsterkningskonstanter Kp, Ki og Kd er
  //    gitt i hundredeler.
  
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
Zumo32U4LCD display;
BranchNode node_array[TOTAL_NODES];
State state = State::RESET;
State state_prev;
PID pid;
Zumo32U4Encoders encoders;


// Globale variabler
bool linesensors_calibrated_since_last_powerup = false;
bool state_has_changed;
uint16_t update_counter;
unsigned long time_in_state;
unsigned long time_0;
unsigned long time_since_lcd_update;
int16_t line_sensor_values[NUM_SENSORS];
int16_t line_position;
char strbuf[4];
char strbuf_8[8];
int32_t max_speed = MAX_SPEED;
int32_t left_speed = 0;
int32_t right_speed = 0;
int32_t lastDisplayTime;
float countsLeft;
float countsRight;
float counts_no_reset;

// PID-regulator-variabler.
// TODO: lag en struct/class for innstilling av PID-regulator så det ikke er så mye rot her
uint16_t adjustment = 0;
uint16_t adjustment_amount = 1;
int16_t error = 0;
int32_t speed_difference;
enum PidTuneState : uint16_t {P, I, D, V} pid_tune_state;



void setup(){
  // TODO: Opprett kommunikasjon med ESP32 på ryggen til zumoen vha. serial
  line_sensors.initFiveSensors();
  update_counter = 0;
}


void loop(){
  // TODO: sende batteridata til ESP32
  // TODO: sende Wall-E's tilstand (state) til ESP32
  // TODO: motta sensornodeinformasjon fra ESP32
  // TODO: motta strømpriser osv fra ESP32


  
  // Operasjoner som gjøres hver gang før tilstandsmaskinen oppdateres
  state_prev = state;
  line_position = line_sensors.readLine(line_sensor_values);
  // TODO: gjør også en linjesensormåling som kan se etter forgreining i stien (??)
  
 if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    //Henter antall counts fra encoder, i løpet av 0.1 sekunder.
    countsLeft = encoders.getCountsAndResetLeft();
    countsRight = encoders.getCountsAndResetRight();
  }

    //Teller antall counts siden programstart
    counts_no_reset = (encoders.getCountsLeft()+ encoders.getCountsRight()) / 2;


  if ((millis() - time_since_lcd_update) > LCD_UPDATE_DELAY_MS) {
    display.clear();
  }

  // Gå til og fra stemming av PID-regulator dersom knapp B og C er trykka ned
  if (button_b.isPressed() && button_c.isPressed()) { state = State::PID_TUNE; }
  


  // Oppdater tilstandsmaskinen
  switch (state) {
    case State::RESET: {
      // Wall-E commit sudoku

      // Resetter "node_array" med null
      memset(&node_array, 0, sizeof(node_array));

      // Kjører kalibrering om det ikke allerede er gjort
      if (!linesensors_calibrated_since_last_powerup){
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
        left_speed, right_speed = 0;
        state = State::WAIT_FOR_START_SIGNAL;
      }
      display.gotoXY(0,0);
      display.print(F("Calib."));
    } break;

    case State::WAIT_FOR_START_SIGNAL: {
      // Wall-E venter på signal om å starte

      // Vent på at noen trykker på knapp A
      if (button_a.getSingleDebouncedPress()) { state = State::MOVING; }
      if (button_b.getSingleDebouncedPress()) { state = State::PID_TUNE; }

      display.gotoXY(0,0);
      display.print(F("A: move"));
      display.gotoXY(0,1);
      display.print(F("B: tune"));

      left_speed = 0;
      right_speed = 0;

      // TODO: aksepter startsignal fra ESP32
    } break;
    
    case State::MOVING: {
      // Wall-E kjører langs en sti

      // TODO: legg til styring vha. PID-regulator her når den fungerer bra
      
      
      // TODO: se om vi er over en forgreining -> gå til BRANCH_FOUND
      // TODO: se om vi har funnet en brusboks (bruk avstandssensorene?)
      // TODO: vurder om vi har nok batteri til å fortsette
      //      - her kan vi også sjekke opp mot data fra nettet om det lønner seg å vente
      
      //funksjon som kjører rundt innenfor en border
      //searching();

      
      // Printer SpeedMeter til LCD:
      
      float avg_speed = speedmeter(countsLeft, countsRight);
      display.gotoXY(0,0);
      display.print(avg_speed);

      //Printer batterinivaa til LCD
      display.gotoXY(0,1);
      display.print(batteryLevel(counts_no_reset));

      if (batteryLevel(counts_no_reset) < LOW_BATTERY){
        // Kjør til ladestasjon
        state = State::RETURN_TO_STATION;
        }
      else if (batteryLevel(counts_no_reset) == 0){
        //dødt batteri
        state = State::STOPPED;
        }
      
      
    } break;

    case State::BRANCH_FOUND: {
      // Wall-E fant en forgreining i stien
      // TODO: sjekk om denne forgreininga allerede er registrert i "node_array",
      //      - hvis ikke, gå til MAP_BRANCHPOINT
      // TODO: velg hvilken vei vi skal gå
    } break;

    case State::MAP_BRANCHPOINT: {
      // Wall-E har funnet en ny forgreining og kartlegger den
      // TODO: rygg litt, snu 90 grader til høgre, kjør en hel runde rundt og gjør målinger
      // TODO: oppdater "node_array" på grunnlag av disse målingene
    } break;

    case State::RETURN_TO_STATION: {
      // Wall-E kjører til ladestasjonen
      // TODO: lag en plan for å komme til ladestasjonen
      //      - i praksis, bruk "node_array" for å finne den raskeste veien hjem
      // TODO: pass på at vi følger linja
    } break;
    
    case State::BRAKING: {
      // Wall-E bremser ned motorene
      // TODO: skru av motorene gradvis over et par-fem runder
    } break;

    case State::STOPPED: {
      // Wall-E står stille
      // TODO: start opp igjen på grunnlag av visse kriterier
    } break;

    case State::REFUELING: {
      // Wall-E er på ladestasjonen og fyller opp batteriene
      // TODO: start opp igjen når batteriene er fulle
    } break;

    case State::SPIRALLING: {
      // Wall-E kjører rundt i en spiral for å se etter linja. Denne tilstanden
      //        brukes både for å treffe en ny forgreining og for å finne tilbake
      //        til stien om Wall-E har mista den.
      // TODO: kjør rundt i stadig større spiral
      // TODO: sjekk måling om vi er over linja igjen
    } break;

    case State::PID_TUNE: {
      // Innstillingsmodus for PID-regulatoren.
      //   - press "A" for å velge parameter
      //   - press "B" eller "C" for å senke eller øke parameteret

      display.gotoXY(0, 0);

      // Endre verdi med "B" og "C"
      adjustment = 0;
      if (button_b.getSingleDebouncedPress()) {adjustment = -adjustment_amount; }
      if (button_c.getSingleDebouncedPress()) {adjustment = adjustment_amount; }
      
      switch (pid_tune_state) {
        case PidTuneState::P: {
          display.print(F("P:"));
          pid.Kp += adjustment;
          sprintf(strbuf, "%d", pid.Kp);
          if (button_a.getSingleDebouncedPress()) { pid_tune_state = PidTuneState::I; }
        } break;
        
        case PidTuneState::I: {
          display.print(F("I:"));
          pid.Ki += adjustment;
          sprintf(strbuf, "%d", pid.Ki);
          if (button_a.getSingleDebouncedPress()) { pid_tune_state = PidTuneState::D; }
        } break;
        
        case PidTuneState::D: {
          display.print(F("D:"));
          pid.Kd += adjustment;
          sprintf(strbuf, "%d", pid.Kd);
          if (button_a.getSingleDebouncedPress()) { pid_tune_state = PidTuneState::V; }
        } break;

        case PidTuneState::V: {
          display.print(F("V:"));
          max_speed += adjustment*10;
          sprintf(strbuf, "%d", max_speed);
          if (button_a.getSingleDebouncedPress()) { pid_tune_state = PidTuneState::P; }
        } break;
      }

      // Print talla til skjermen
      display.gotoXY(3, 0);
      display.print(strbuf);
      display.gotoXY(0, 1);
      sprintf(strbuf_8, "%d", error);
      display.print(strbuf_8);

      // Regn ut PID-transferfunksjon
      error = line_position - 2000;
      speed_difference = pid.GetOutput(error);
    
      left_speed = constrain((max_speed + speed_difference), -max_speed/3, max_speed);
      right_speed = constrain((max_speed - speed_difference), -max_speed/3, max_speed);
    
      } break;
  }

  // Operasjoner som gjøres hver gang etter tilstandsmaskinen oppdateres
  state_has_changed = (state_prev != state);
  if (state_has_changed) { time_in_state = millis(); }
  

  motors.setSpeeds(left_speed, right_speed);
  if ((millis() - time_since_lcd_update) > LCD_UPDATE_DELAY_MS) {
    display.display();
    time_since_lcd_update = millis();
  }

  // TODO: basert på ny informasjon kan vi oppdatere "node_array" her

  // Plusser på "1" til "update_counter", med mindre den er MAX_INT, da går den til 0
  update_counter = (update_counter == 65535) ? 0 : update_counter + 1;
}
