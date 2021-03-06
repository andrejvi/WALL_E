#pragma once

#include "ZumoState.h"

/*
   Packet.h    ---> datastruktur for pakke som sendes over seriell- og radiokommunikasjon.

   Idéer for framtidig innhold:
    - Batteritilstand (lader ut eller lader opp)
    - Batteristatus (ladningsforhold)
    - PID-reguleringskonstanter, Kp, Ki, Kd
    - Systemstatus (kjøretid, kjørelengde [cm], temperatur i MCU-kjerne)
*/

typedef struct Package {
  // Byte som indikerer at melding starter. Denne setter vi til "<" i Zumo.ino
  uint8_t start_byte;
  

  // Innholdet i pakka
  ZumoState zumo_state;
  uint8_t Kp;
  uint8_t Ki;
  uint8_t Kd;
  uint16_t battery_level;
  uint16_t speed;
  float ultrasonic_distance_reading;

  // Flagg. "1" indikerer ny data for verdi,
  //        "0" indikerer at data for denne verdien skal ignoreres
  bool update_zumo_state;
  bool update_Kp;
  bool update_Ki;
  bool update_Kd;

  uint16_t battery_real;
  uint16_t bank_balance;

  // Byte som indikerer at melding stopper. Denne setter vi til ">" i Zumo.ino
  uint8_t stop_byte;
} Package;
