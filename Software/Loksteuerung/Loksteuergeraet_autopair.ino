/*
  Loksteuergerät Version 0.9.2 RC3
  Software für Loksteuerung über ESP-NOW
  Andreas Hauschild 2026
  Prozessor: ESP32-WROOM-DA
  Messwerte UBatt:  12.7V: 1575   11.5V: 1410
                    25.4V: 2955   23,0V: 2750
  mit Autopair-Funktion, damit keine MAC mehr notwendig                   
*/

#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- KONFIGURATION PINS ---
const int Licht_Rueck_F_Pin = 12; // VORSICHT: Boot-Pin (MTDI)
const int Licht_Rueck_R_Pin = 13;
const int Licht_Front_R_Pin = 14;
const int Licht_Front_F_Pin = 27;
const int TempSensor_Pin    = 0;  // VORSICHT: Boot-Pin (GPIO0)
const int XLC_6_1_Pin       = 4; 
const int XLC_6_2_Pin       = 16;
const int _4QD_POT_Pin      = 25;
const int _4QD_ON_Pin       = 26;
const int _4QD_DIR_Pin      = 33;
const int Horn_Pin          = 32;
const int Hallsensor_Pin    = 35;
const int UBatt_Pin         = 34;

// Liste für Initialisierung
const int ALL_OUTPUTS[] = {12, 13, 14, 27, 4, 16, 26, 33, 32};
const int NUM_OUTPUTS = 9;

// --- DATENSTRUKTUREN (Sichere Syntax für GCC Compiler) ---
#pragma pack(push, 1)
typedef struct {
  uint8_t id; uint8_t version; uint8_t Fahrschalter;
  bool Hptsch_ON; uint8_t Richtung; uint8_t Bremse;
  uint8_t Licht_F; uint8_t Licht_R; bool Licht_Fst;
  bool Panto_F; bool Panto_R; bool Horn; bool Sound_ON;
  bool Sound_1; bool Sound_2; bool Sound_3; bool Sound_4;
} FST_message;

typedef struct {
  uint8_t id; uint8_t version; int UBatt; int IMot1;
  int IMot2; int Speed; int Temp; int RSSI;
} LST_message;

typedef struct {
  uint8_t msgType; uint8_t macAddr[6];
} Pairing_Message;
#pragma pack(pop)

// --- GLOBALE VARIABLEN ---
uint8_t senderMAC[6] = {0,0,0,0,0,0};
bool isPaired = false;
unsigned long RXTimer = 0;
const unsigned long RXInterval = 2000;
bool RXSuccess = false;
bool E_Stop = false;

int _4QD_ON = 0, _4QD_DIR = 0, _4QD_POT = 0, Horn = 0;
int Licht_Front_F = 0, Licht_Front_R = 0, Licht_Rueck_F = 0, Licht_Rueck_R = 0;

unsigned long HornTimer = 0;
const unsigned long HornInterval = 500;
unsigned long E_BlinkTimer = 0;
const unsigned long E_BlinkInterval = 1000;
int E_BlinkCount = 0;
bool E_BlinkState = false;
unsigned long lastUpdate = 0;
const unsigned long updateInterval = 500;

FST_message RXData;
LST_message TXData;
Preferences preferences;
OneWire oneWire(TempSensor_Pin);
DallasTemperature sensors(&oneWire);

// --- FUNKTIONEN ---

void updateHardware() {
  digitalWrite(Horn_Pin, Horn);
  digitalWrite(Licht_Front_F_Pin, Licht_Front_F);
  digitalWrite(Licht_Front_R_Pin, Licht_Front_R);
  digitalWrite(Licht_Rueck_F_Pin, Licht_Rueck_F);
  digitalWrite(Licht_Rueck_R_Pin, Licht_Rueck_R);

  if (!E_Stop) {
    digitalWrite(_4QD_ON_Pin, _4QD_ON);
    digitalWrite(_4QD_DIR_Pin, _4QD_DIR);
    analogWrite(_4QD_POT_Pin, _4QD_POT);
  } else {
    analogWrite(_4QD_POT_Pin, 0);
    digitalWrite(_4QD_ON_Pin, LOW);
  }
}

void EmergencyStop() {
  if (!E_Stop) {
    E_Stop = true; 
    HornTimer = millis(); 
    Horn = true;
    E_BlinkTimer = millis(); 
    E_BlinkState = true; 
    E_BlinkCount = 0;
  }
  _4QD_POT = 0; 
  _4QD_ON = LOW;
  
  if (millis() - HornTimer > HornInterval) Horn = false;
  
  if (E_BlinkCount < 10) {
    if (millis() - E_BlinkTimer > E_BlinkInterval) {
      E_BlinkTimer = millis(); 
      E_BlinkState = !E_BlinkState; 
      E_BlinkCount++;
    }
    Licht_Front_R = Licht_Rueck_R = E_BlinkState;
  } else {
    Licht_Front_R = Licht_Rueck_R = HIGH;
  }
  Licht_Front_F = Licht_Rueck_F = LOW;
}

float readUBatt() {
  long sum = 0;
  for(int i=0; i<10; i++) { sum += analogRead(UBatt_Pin); delay(2); }
  return (float)(sum / 10);
}

// Callback für ESP32 Core 3.x
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
  const uint8_t *mac_addr = info->src_addr;

  if (len == sizeof(Pairing_Message)) {
    Pairing_Message pMsg;
    memcpy(&pMsg, incomingData, sizeof(Pairing_Message));
    if (pMsg.msgType == 1) {
      if (isPaired && memcmp(mac_addr, senderMAC, 6) == 0) {
        Pairing_Message resp; resp.msgType = 2;
        WiFi.macAddress(resp.macAddr);
        esp_now_send(mac_addr, (uint8_t *)&resp, sizeof(resp));
        return;
      }
      preferences.begin("lok_config", false);
      preferences.putBytes("senderMAC", mac_addr, 6);
      preferences.end();
      delay(200);
      ESP.restart();
    }
    return;
  }

  if (len == sizeof(FST_message)) {
    if (isPaired && memcmp(mac_addr, senderMAC, 6) == 0) {
      memcpy(&RXData, incomingData, sizeof(FST_message));
      RXSuccess = true;
      RXTimer = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- LOK START ---");

  for (int i=0; i < NUM_OUTPUTS; i++) {
    pinMode(ALL_OUTPUTS[i], OUTPUT);
    digitalWrite(ALL_OUTPUTS[i], LOW);
  }

  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.disconnect();
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Error");
    delay(2000);
    ESP.restart();
  }

  // WICHTIG: Expliziter Cast gegen "warnings as errors"
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);

  preferences.begin("lok_config", true);
  if (preferences.getBytesLength("senderMAC") == 6) {
    preferences.getBytes("senderMAC", senderMAC, 6);
    if (senderMAC[0] != 0xFF && senderMAC[0] != 0x00) {
      isPaired = true;
      esp_now_peer_info_t peer = {};
      memcpy(peer.peer_addr, senderMAC, 6);
      esp_now_add_peer(&peer);
      Serial.println("MAC geladen.");
    }
  }
  preferences.end();

  analogWriteFrequency(_4QD_POT_Pin, 20000);
  sensors.begin();
}

void loop() {
  // 1. Funk-Check
  if (millis() - RXTimer > RXInterval) {
    EmergencyStop();
  }

  // 2. Datenverarbeitung
  if (RXSuccess) {
    RXSuccess = false;
    E_Stop = false;
    
    _4QD_POT = map(constrain(RXData.Fahrschalter, 0, 99), 0, 99, 0, 255);
    _4QD_ON = RXData.Hptsch_ON;
    _4QD_DIR = (RXData.Richtung == 2) ? HIGH : LOW;
    if (RXData.Richtung == 0) _4QD_ON = LOW;

    Licht_Front_F = (RXData.Licht_F > 0);
    Licht_Rueck_R = (RXData.Licht_F == 2);
    Licht_Rueck_F = (RXData.Licht_R > 0);
    Licht_Front_R = (RXData.Licht_R == 2);
    
    Horn = RXData.Horn;

    // Rückmeldung an Sender
    if (isPaired && (millis() - lastUpdate > updateInterval)) {
      TXData.UBatt = (int)readUBatt();
      sensors.requestTemperatures();
      float t = sensors.getTempCByIndex(0);
      TXData.Temp = (t > -50) ? (int)t : 0;
      esp_now_send(senderMAC, (uint8_t *)&TXData, sizeof(TXData));
      lastUpdate = millis();
    }
  }

  updateHardware();
}