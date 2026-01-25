/*
  Loksteuergerät Version 0.9
  Software für Loksteuerung über ESP-NOW
  Andreas Hauschild 2026
  Prozessor: ESP32-WROOM-DA
  Messwerte UBatt:  12.7V: 1575   11.5V: 1410
                    25.4V: 2955   23,0V: 2750
*/

#include <Wire.h>
#include <ESP32_NOW.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <OneWire.h>
#include <DallasTemperature.h>

// MAC-Adresse des Senders
uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0xC2, 0x3D, 0xB5};

// Pin-Definitionen
const int Licht_Rueck_F_Pin = 12, Licht_Rueck_R_Pin = 13, Licht_Front_R_Pin = 14, Licht_Front_F_Pin = 27;
const int TempSensor_Pin = 0; // DS18B20 Datenleitung
const int XLC_6_1_Pin = 4, XLC_6_2_Pin = 16;
const int _4QD_POT_Pin = 25, _4QD_ON_Pin = 26, _4QD_DIR_Pin = 33;
const int Horn_Pin = 32, Hallsensor_Pin = 35, UBatt_Pin = 2;

const int outputPins[] = {12, 13, 14, 27, 4, 16, 26, 33, 32}; 

// Klassen-Instanzen für Temperatur
OneWire oneWire(TempSensor_Pin);
DallasTemperature sensors(&oneWire);

long RXTimer, RXInterval = 2000;
bool RXSuccess, E_Stop = false;
int _4QD_ON, _4QD_DIR, _4QD_POT, Horn;
int Licht_Front_F, Licht_Front_R, Licht_Rueck_F, Licht_Rueck_R;
long lastRSSI, HornTimer, HornInterval = 500;
long E_BlinkTimer, E_BlinkInterval = 1000;
int E_BlinkCount = 0;
bool E_BlinkState = false;
unsigned long lastUpdate = 0;
const long updateInterval = 5000;

typedef struct FST_message {
  uint8_t id, version, Fahrschalter;
  bool Hptsch_ON;
  uint8_t Richtung, Bremse, Licht_F, Licht_R;
  bool Licht_Fst, Panto_F, Panto_R, Horn, Sound_ON, Sound_1, Sound_2, Sound_3, Sound_4;
} __attribute__((packed)) FST_message;

typedef struct LST_message {
  uint8_t id = 100, version = 1;
  int UBatt, IMot1, IMot2, Speed, Temp, RSSI;
} __attribute__((packed)) LST_Message;

FST_message RXData;
LST_message TXData;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  lastRSSI = recv_info->rx_ctrl->rssi;
  memcpy(&RXData, incomingData, sizeof(RXData));
  RXSuccess = true;
  RXTimer = millis();
}

float readUBatt() {
  WiFi.disconnect(); 
  esp_wifi_stop();
  delay(10);
  int raw = analogRead(UBatt_Pin);
  esp_wifi_start();
  return raw;
}

void EmergencyStop() {
  if (!E_Stop) {
    E_Stop = true; HornTimer = millis(); Horn = true;
    E_BlinkTimer = millis(); E_BlinkState = true; E_BlinkCount = 0;
  }
  _4QD_POT = 0;
  if (millis() - HornTimer > HornInterval) Horn = false;
  if (E_BlinkCount < 10) {
    if (millis() - E_BlinkTimer > E_BlinkInterval) {
      E_BlinkTimer = millis(); E_BlinkState = !E_BlinkState; E_BlinkCount++;
    }
    Licht_Front_R = Licht_Rueck_R = E_BlinkState;
  } else {
    Licht_Front_R = Licht_Rueck_R = HIGH;
  }
  Licht_Front_F = Licht_Rueck_F = LOW;
}

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
  }
}

void setup() {
  Serial.begin(115200);
  for (int pin : outputPins) { pinMode(pin, OUTPUT); digitalWrite(pin, 0); }
  analogWriteFrequency(25, 20000);
  
  sensors.begin(); // DS18B20 Start

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  esp_now_add_peer(&peerInfo);
}

void loop() {
  if (millis() - RXTimer > RXInterval) EmergencyStop();

  if (RXSuccess) {
    RXSuccess = false;
    _4QD_POT = map(constrain(RXData.Fahrschalter, 0, 99), 0, 99, 0, 255);
    _4QD_ON = RXData.Hptsch_ON;
    _4QD_DIR = (RXData.Richtung == 2);
    if (RXData.Richtung == 0) _4QD_ON = LOW;

    Licht_Front_F = (RXData.Licht_F > 0);
    Licht_Rueck_R = (RXData.Licht_F == 2);
    Licht_Rueck_F = (RXData.Licht_R > 0);
    Licht_Front_R = (RXData.Licht_R == 2);
    Horn = RXData.Horn;
    E_Stop = false;

    if (millis() - lastUpdate > updateInterval) {
      TXData.UBatt = readUBatt();
      sensors.requestTemperatures();
      float t = sensors.getTempCByIndex(0);
      TXData.Temp = (t > -50) ? (int)t : 0;
      TXData.RSSI = (lastRSSI > -50) ? 4 : (lastRSSI > -65) ? 3 : (lastRSSI > -75) ? 2 : (lastRSSI > -85) ? 1 : 0;
      lastUpdate = millis();
    }
    esp_now_send(broadcastAddress, (uint8_t *)&TXData, sizeof(TXData));
  }
  updateHardware();
}