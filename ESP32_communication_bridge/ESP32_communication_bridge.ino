#include <esp_now.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();


// VIKTIG! Denne linja er alt du trenger å endre for å flashe de to forskjellige ESP'ene.
#define IS_ZUMO     // Kommenter ut for å flashe RPi-ESP, behold den for å flashe ZUMO-ESP.


/*
   Kommunikasjonsoppsett: [ZUMO <--(serie)--> ESP <--(radio)--> ESP <--(serie)--> RPi]

    Vi bruker samme pakkeformat over alt, som definert i "Package.h".
    Systemet er agnostisk mtp. MASTER<-->SLAVE, og det er seriekommunikasjonsenhetene
    (Raspberry Pi og ZUMO32u4) som bestemmer hvorvidt noe skal endres i pakken.
    Vanligvis vil ZUMOen kun rapportere status, mens en pakke fra Raspberry Pi-sida
    vil forespør en endring. Denne fila er ikke ansvarlig for denne logikken.

    Send over seriell når...
      Når ZUMO-ESP mottar pakke fra radio --> send videre til ZUMO over seriell
      Når RPI-ESP  mottar pakke fra radio --> send videre til RPi over seriell

    Send over "ESP Now" når...
      ZUMO-ESP har mottatt ny pakke fra seriell --> send til RPi over radio
      RPi-ESP  har mottatt ny pakke fra seriell --> send til ZUMO over radio

    Hardware: TTGO ESP32 med innebygd skjerm som drives av TFT_eSPI-biblioteket.

*/


#ifdef IS_ZUMO
uint8_t PEER_MAC_ADDR[] = {0x84, 0xCC, 0xA8, 0x61, 0x51, 0x28}; // (ESP med rett resetknapp)
#else
uint8_t PEER_MAC_ADDR[] = {0x84, 0xCC, 0xA8, 0x61, 0x56, 0x88}; // (ESP med skeiv resetknapp)
#endif


// TODO: lag headerfil for denne -- ZumoState.h
enum ZumoState : uint8_t {
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


// TODO: lag headerfil for denne -- Package.h
/*
   Package.h    ---> datastruktur for pakke som sendes over seriell- og radiokommunikasjon.

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



// TODO: denne burde vel også legges til i ZumoState.h
String zumo_state_to_str(ZumoState state) {
  switch (state) {
    case ZumoState::RESET:
      return "RESET";
    case ZumoState::CALIBRATE_LINESENSORS:
      return "CALIBRATE_LINESENSORS";
    case ZumoState::WAIT_FOR_START_SIGNAL:
      return "WAIT_FOR_START_SIGNAL";
    case ZumoState::MOVING:
      return "MOVING";
    case ZumoState::BRANCH_FOUND:
      return "BRANCH_FOUND";
    case ZumoState::MAP_BRANCHPOINT:
      return "MAP_BRANCHPOINT";
    case ZumoState::RETURN_TO_STATION:
      return "RETURN_TO_STATION";
    case ZumoState::BRAKING:
      return "BRAKING";
    case ZumoState::STOPPED:
      return "STOPPED";
    case ZumoState::REFUELING:
      return "REFUELING";
    case ZumoState::SPIRALLING:
      return "SPIRALLING";
    case ZumoState::PID_TUNE:
      return "PID_TUNE";

    default:
      return "unknown state";
  }
}


// Globale objekter
esp_now_peer_info_t peerInfo;
Package serial_received_package;
Package radio_received_package;


// Globale variabler
unsigned long timer_start = 0;
bool require_screen_update = true;

// Feil- og statusmeldingsvariabler
esp_err_t esp_now_init_status;
esp_err_t peer_add_status;
esp_err_t register_send_callback_status;
esp_err_t register_recv_callback_status;
esp_now_send_status_t last_send_status;



void esp_now_on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // "Callback"-funksjonen som kjøres hver gang "esp_now_send" kjøres og vi sender
  // ei pakke over radiokommunikasjon (ESP Now)
  
  last_send_status = status;
  require_screen_update = true;
}


void esp_now_on_data_receive(const uint8_t *mac, const uint8_t *incoming_data, int bytes_received) {
  // "Callback"-funksjon som kjøres når vi mottar ei pakke over radiokommunikasjon.
  
  memcpy(&radio_received_package, incoming_data, sizeof(radio_received_package));
  require_screen_update = true;

  // FIXME: her vil zumoen endre tilstand og bekrefte med ny serialpakke. Siden ZUMOen
  // er på påskeferie er det kun en enkel loopback her nå.
#ifdef IS_ZUMO
  serial_received_package.zumo_state = radio_received_package.zumo_state;
  esp_err_t last_send_result = esp_now_send(PEER_MAC_ADDR, (uint8_t *) &serial_received_package, sizeof(serial_received_package));
#endif

}



void update_display() {
  // All skriving til skjermen gjøres her. Fikk tidligere masse kræsjer når skrivinga
  // skjedde rundt omkring i koden, så dette virker som en bedre løsning.
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0, 2);

#ifdef IS_ZUMO
  tft.println("ZUMO-ESP");
#else
  tft.println("RASPBERRY-ESP");
#endif

  tft.print("MAC: ");
  tft.println(WiFi.macAddress());

  if (peer_add_status != ESP_OK) {
    tft.println("Failed to add peer");
  }

  tft.print("Send Status: ");
  switch (last_send_status) {
    case ESP_NOW_SEND_SUCCESS:
      tft.setTextColor(TFT_GREEN);
      tft.println("sent and received ACK");
      tft.setTextColor(TFT_WHITE);
      break;
    case ESP_NOW_SEND_FAIL:
      tft.setTextColor(TFT_RED);
      tft.println("no ACK received");
      tft.setTextColor(TFT_WHITE);
      break;
  }

#ifdef IS_ZUMO
  tft.print("zumo state: ");
  tft.println(zumo_state_to_str(serial_received_package.zumo_state));
  tft.print("requested: ");
  tft.println(zumo_state_to_str(radio_received_package.zumo_state));
#else

  tft.print("zumo state: ");
  tft.println(zumo_state_to_str(radio_received_package.zumo_state));
  tft.print("requested: ");
  tft.println(zumo_state_to_str(serial_received_package.zumo_state));
#endif

}


bool receive_new_serial_package() {
  // Sjekk om det kommer inn pakke over serieporten, og oppdater
  // "serial_received_package" dersom det kommer inn ei pakke.


  /*
    if (Serial.available() > 0) {
      uint8_t serial_incoming_buffer[sizeof(serial_received_package)] = {Serial.read()};
      Package* serial_received_package = (Package*) serial_incoming_buffer;
      return true;
    } else {
      return false;
    }
  */

  // FIXME: Akkurat nå tar vi kun i mot en byte som tilsvarer ZumoState (som er en uint8_t).
  //        Dette vil kun funke så lenge pakka bare inneholder én byte.
  if (Serial.available() > 0) {
    serial_received_package.zumo_state = (ZumoState)Serial.read();
    return true;
  } else {
    return false;
  }
}


void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);

  pinMode(0, INPUT_PULLUP); // Knapp 0 for debugging

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init esp now
  esp_now_init_status = esp_now_init();


  // Legg til den andre ESP'en som "peer"
  memcpy(&peerInfo.peer_addr, PEER_MAC_ADDR, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peer_add_status = esp_now_add_peer(&peerInfo);


  // Registrerer "callback" for radiokommunikasjon
  register_send_callback_status = esp_now_register_send_cb(esp_now_on_data_sent);
  register_recv_callback_status = esp_now_register_recv_cb(esp_now_on_data_receive);
}


void loop() {
  // Siden ESP Now bruker svart magi (interrupts) for å kalle funksjonen "esp_now_on_data_receive"
  // når enn ei pakke dukker opp over radiokommunikasjon kan det være vanskelig å følge
  // programflyten her, og vi kan også få merkelige kræsjer pga. skriving til de samme
  // minneområdene. Men etter stresstesting (endra tilstand hvert 10 ms) virker det som om
  // denne løsningen er robust nok for hva vi skal gjøre.

  
  if (receive_new_serial_package()) {
    // Kun om vi har fått ei ny pakke over serieporten vil det sendes ny pakke over radio
    esp_err_t last_send_result = esp_now_send(PEER_MAC_ADDR, (uint8_t *) &serial_received_package, sizeof(serial_received_package));
  }


  if (require_screen_update) {
    update_display();
    require_screen_update = false;
  }
}
