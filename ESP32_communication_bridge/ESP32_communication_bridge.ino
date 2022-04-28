#include <esp_now.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include "ZumoState.h"
#include "Packet.h"    // Disse må kopieres manuelt over i denne mappa..



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

// Pins for seriekommunikasjon ZUMO<--->ESP
#define TX_ZUMO_ESP 17
#define RX_ZUMO_ESP 2

// Konfigurerer pakke
#define PACKAGE_START_BYTE 0b00111100   // "<"
#define PACKAGE_STOP_BYTE  0b00111110   // ">"


#ifdef IS_ZUMO
uint8_t PEER_MAC_ADDR[] = {0x84, 0xCC, 0xA8, 0x61, 0x51, 0x28}; // (ESP med rett resetknapp)
#else
uint8_t PEER_MAC_ADDR[] = {0x84, 0xCC, 0xA8, 0x61, 0x56, 0x88}; // (ESP med skeiv resetknapp)
#endif

// Konstanter
const uint8_t PACKAGE_SIZE = sizeof(Package);


// Globale objekter
TFT_eSPI tft = TFT_eSPI();
esp_now_peer_info_t peerInfo;
uint8_t serial_buffer[PACKAGE_SIZE];
Package serial_received_package;
Package radio_received_package;


// Globale variabler
unsigned long timer_start = 0;
bool require_screen_update = true;
char strbuf[16];

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

  // Sender den nylig mottatte esp-now--pakka videre over serial
  Serial.write((byte*)&radio_received_package, PACKAGE_SIZE);
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

  tft.print("PID: Kp: ");  tft.print(serial_received_package.Kp);
  tft.print(",  Ki: ");    tft.print(serial_received_package.Ki);
  tft.print(",  Kd: ");    tft.println(serial_received_package.Kd);

  //tft.print("Batt.: ");  tft.print(serial_received_package.battery_level);
  //tft.print(", Speed: "); tft.println(serial_received_package.speed);
  tft.print(", Dist.: "); tft.println(serial_received_package.ultrasonic_distance_reading);
#else

  tft.print("zumo state: ");
  tft.println(zumo_state_to_str(radio_received_package.zumo_state));
  tft.print("requested: ");
  tft.println(zumo_state_to_str(serial_received_package.zumo_state));

  tft.print("PID: Kp: ");  tft.print(radio_received_package.Kp);
  tft.print(",  Ki: ");    tft.print(radio_received_package.Ki);
  tft.print(",  Kd: ");    tft.println(radio_received_package.Kd);

  //tft.print("Batt.: ");  tft.print(radio_received_package.battery_level);
  //tft.print(", Speed: "); tft.print(radio_received_package.speed);
  tft.print(", Dist.: "); tft.println(radio_received_package.ultrasonic_distance_reading);
#endif

  tft.print("PKG: ");
  for (int i = 0; i < PACKAGE_SIZE - 1; i++) {
    tft.print(serial_buffer[i], HEX);
    tft.print(" ");
  }
}


bool receive_serial_package() {
  // Returnerer enten "false" om vi ikke har fått inn pakke, eller
  // "true" dersom en ny pakke er skrevet over på "serial_received_package".

  static bool receive_in_progress = false;
  static byte index = 0;
  uint8_t received_byte;

  while (Serial.available() > 0) {
    received_byte = Serial.read();

    if (received_byte == PACKAGE_START_BYTE) {
      receive_in_progress = true;
    }

    if (receive_in_progress) {
      if (received_byte != PACKAGE_STOP_BYTE) {
        serial_buffer[index] = received_byte;
      }

      if (received_byte == PACKAGE_STOP_BYTE) {
        receive_in_progress = false;
        serial_buffer[index] = received_byte;
        index = 0;

        // Vi har mottatt pakke, kopierer nå over i "serial_received_package"
        memset(&serial_received_package, 0, PACKAGE_SIZE);
        memcpy(&serial_received_package, serial_buffer, PACKAGE_SIZE);
        return true;
      }

      if (index > PACKAGE_SIZE) {
        // Vi har ikke fått noen "stop_byte" etter mange mottatte bytes, noe er galt
        return false;
      }

      index ++;
    }
  }
  return false;
}


void setup() {
#ifdef IS_ZUMO
  Serial.begin(115200, SERIAL_8N1, RX_ZUMO_ESP, TX_ZUMO_ESP);
#else
  Serial.begin(115200);
#endif

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


  if (receive_serial_package()) {
    // Kun om vi har fått ei ny pakke over serieporten vil det sendes ny pakke over radio
    esp_err_t last_send_result = esp_now_send(PEER_MAC_ADDR, (uint8_t *) &serial_received_package, sizeof(serial_received_package));
  }


  if (require_screen_update) {
    update_display();
    require_screen_update = false;
  }
}
