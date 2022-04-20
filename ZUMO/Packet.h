#pragma once

#include "ZumoState.h"
/*
   Packet.h    ---> datastruktur for pakke som sendes over seriell- og radiokommunikasjon.

   Idéer for framtidig innhold:
    - Batteritilstand (lader ut eller lader opp)
    - Batteristatus (ladningsforhold)
    - PID-reguleringskonstanter, Kp, Ki, Kd
    - Systemstatus (kjøretid, kjørelengde [cm], temperatur i MCU-kjerne)
    -
*/
typedef struct Package {
  ZumoState zumo_state;
} Package;