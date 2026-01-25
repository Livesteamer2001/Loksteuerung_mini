/*
  Loksteuergerät Version 0.8
  Software für Loksteuerung über ESP-NOW
  Andreas Hauschild 2026
  Prozessor: ESP32-WROOM-DA
  Board-Version: esp32 Version 3.3.3
  Messwerte UBatt:  12.7V: 1575   11.5V: 1410
                    25.4V: 2955   23,0V: 2750



*/

#include <Wire.h>
#include <ESP32_NOW.h>
#include <WiFi.h>
#include "esp_wifi.h"

// Eigene MAC-Adresse
//uint8_t broadcastAddress[] = {0x88, 0x57, 0x21, 0xBF, 0x56, 0x98};
// MAC-Adresse des Senders
uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0xC2, 0x3D, 0xB5};

// Pin-Definitionen
const int Licht_Rueck_F_Pin = 12;   //
const int Licht_Rueck_R_Pin = 13;
const int Licht_Front_R_Pin = 14;
const int Licht_Front_F_Pin = 27;
const int TempSensor_Pin = 0;
const int XLC_6_1_Pin = 4;
const int XLC_6_2_Pin = 16;
const int _4QD_POT_Pin = 25;    // der "_" am Anfang ist wegen C-Syntax notwendig!
const int _4QD_ON_Pin = 26;
const int _4QD_DIR_Pin = 33;
const int Horn_Pin = 32;
const int Hallsensor_Pin = 35;
const int UBatt_Pin = 2;

const int outputPins[] = {
  Licht_Rueck_F_Pin,   //
  Licht_Rueck_R_Pin,
  Licht_Front_R_Pin,
  Licht_Front_F_Pin,
  TempSensor_Pin,
  XLC_6_1_Pin,
  XLC_6_2_Pin,
  _4QD_ON_Pin,
  _4QD_DIR_Pin,
  Horn_Pin,
  Hallsensor_Pin 
};

const int TX_id = 100;
const int RX_id = 100;
long RXTimer;
long RXInterval = 2000;
bool RXSuccess;
unsigned long lastWifiUpdate = 0;
int _4QD_ON = LOW;
int  _4QD_DIR = LOW;   // Front: false, Rück: true
int _4QD_POT = 0;
int  Licht_Front_F = LOW;
int  Licht_Front_R = LOW;
int  Licht_Rueck_F = LOW;
int  Licht_Rueck_R = LOW;
int  Horn = LOW;
bool E_Stop = false;
long HornTimer = 0;
long HornInterval = 500;
long lastRSSI = 0; // Globale Variable für die Anzeige des RSSI
// Blink-Logik für Emergency
int E_BlinkCount = 0;
long E_BlinkTimer = 0;
const int E_BlinkInterval = 1000; // Geschwindigkeit des Blinkens (ms)
bool E_BlinkState = false;
unsigned long lastBatteryUpdate = 0;
const long batteryInterval = 5000; // 5 Sekunden in Millisekunden

typedef struct FST_message {
  uint8_t id;           // Identifizierung, sicher ist sicher...
  uint8_t version;
  uint8_t Fahrschalter = 0; // Potiwert, umgesetzt auf 0 - 99   
  bool Hptsch_ON = false;   // Hauptschalter
  uint8_t Richtung = 0;    // Richtungswahl: 0 Neutral | 1 Front | 2 Rück
  uint8_t Bremse;       // tbd
  uint8_t Licht_F = 0;      // 1 Frontlicht  2 Rücklicht
  uint8_t Licht_R = 0;      // 1 Frontlicht  2 Rücklicht
  bool Licht_Fst;
  bool Panto_F;      
  bool Panto_R;
  bool Horn = false;
  bool Sound_ON;
  bool Sound_1;
  bool Sound_2;
  bool Sound_3;
  bool Sound_4;
} __attribute__((packed)) FST_message;

typedef struct LST_message {
  uint8_t id = 100;           // Identifizierung, sicher ist sicher...
  uint8_t version = 1;
  int UBatt;        // Batteriespannung, 10 Bit
  int IMot1;        // Motorstrom 1, 10 Bit
  int IMot2;        // Motorstrom 2, 10 Bit
  int Speed;        // Fahrgeschwindigkeit
  int Temp;         // Motortemperatur
  int RSSI;         // Empfangsfeldstärke, gespiegelt weil esp32 das einfacher kann
} __attribute__((packed)) LST_Message;

FST_message RXData;
LST_message TXData;

// Variable to store if sending data was successful
bool TXSuccess;
int TXFailCounter = 0;



// Callback when data is sent
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS){
    Serial.println("Delivery success");
    TXSuccess = true;
    TXFailCounter = 0;
  }
  else{
    Serial.println("Delivery fail");
    TXSuccess = false;
    TXFailCounter++;
    if (TXFailCounter > 1000) TXFailCounter = 1000;
  }
}

// Callback when data is received
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  // 1. RSSI aus den Empfangs-Metadaten auslesen
  lastRSSI = recv_info->rx_ctrl->rssi;

  // 2. Daten kopieren
  memcpy(&RXData, incomingData, sizeof(RXData));
  
  // Debug-Ausgabe
  Serial.print("Bytes empfangen: "); Serial.println(len);
  //Serial.print(" | RSSI: "); Serial.print(lastRSSI); 
  //Serial.println(" dBm");

  RXSuccess = true;
  RXTimer = millis();
}

void showRX() {
  String msg = "Hauptsch.: ";
	msg += RXData.Hptsch_ON;
	msg += " | Richt.: ";
	msg += RXData.Richtung;
	msg += " | Licht_F: ";
	msg += RXData.Licht_F; 
	msg += " | Licht_R: ";
	msg += RXData.Licht_R; 
	msg += " | Horn: ";
	msg += RXData.Horn; 
	msg += " | Fahrs.: ";
	msg += RXData.Fahrschalter; 
  Serial.println(msg);
  
  msg = "Licht_F_F: ";
	msg += Licht_Front_F;
	msg += " | Licht_F_R: ";
	msg += Licht_Front_R;
	msg += " | Licht_R_F: ";
	msg += Licht_Rueck_F; 
	msg += " | Licht_R_R: ";
	msg += Licht_Rueck_R;  
	msg += " | UBatt: ";
	msg += TXData.UBatt; 
  msg += " | lastRSSI: ";
  msg += lastRSSI;
	msg += " | RSSI: ";
	msg += TXData.RSSI; 
  Serial.println(msg);
}

void getMessage() {
  if (RXSuccess == true){
    RXSuccess = false;       // warten auf die nächste Message
    int safePot = constrain(RXData.Fahrschalter, 0, 99);
    _4QD_POT = map (safePot, 0, 99, 0, 255);
    _4QD_ON = RXData.Hptsch_ON;
    switch (RXData.Richtung) {
      case 1:
        _4QD_DIR = LOW;
        break;
      case 2:
        _4QD_DIR = HIGH;
        break;
      case 0:
        _4QD_ON = LOW;    // zur Sicherheit, sollte normalerweise nicht passieren
        break;
      default:
        _4QD_ON = LOW;
        break;
    }
    // --- Logik für Fahrtrichtung VORNE (Schalter Frontseite) ---
    switch (RXData.Licht_F) {
      case 0: // Alles Aus
        Licht_Front_F = LOW;
        Licht_Rueck_R = LOW; // Das rote Licht auf der Rückseite
        break;
      case 1: // Nur Weiß vorne
        Licht_Front_F = HIGH;
        Licht_Rueck_R = LOW;
        break;
      case 2: // STRECKENFAHRT: Weiß vorne UND Rot hinten
        Licht_Front_F = HIGH;
        Licht_Rueck_R = HIGH; 
        break; 
      default:
        break;  
    }

    // --- Logik für Fahrtrichtung RÜCKWÄRTS (Schalter Rückseite) ---
    switch (RXData.Licht_R) {
      case 0: // Alles Aus
        Licht_Rueck_F = LOW;
        Licht_Front_R = LOW; // Das rote Licht auf der Frontseite
        break;
      case 1: // Nur Weiß hinten
        Licht_Rueck_F = HIGH;
        Licht_Front_R = LOW;
        break;
      case 2: // STRECKENFAHRT RÜCK: Weiß hinten UND Rot vorne
        Licht_Rueck_F = HIGH;
        Licht_Front_R = HIGH; 
        break; 
      default:
        break;  
    }
    Horn = RXData.Horn;
    E_Stop = false;
    esp_now_send(broadcastAddress, (uint8_t *) &TXData, sizeof(TXData));  // Messwerte senden
showRX();
  }
  
}

void prepareMessage() {
  if (millis() - lastBatteryUpdate > batteryInterval) {
    TXData.UBatt = readUBatt(); 
    lastBatteryUpdate = millis();
    Serial.print("Batterie gemessen: ");
    Serial.println(TXData.UBatt);
  }

  TXData.RSSI = getSignalStrength();
}
int getSignalStrength() {
  if (lastRSSI > -50) return 4;
  if (lastRSSI > -65) return 3;
  if (lastRSSI > -75) return 2;
  if (lastRSSI > -85) return 1;
  return 0;
} 

float readUBatt() {             // Workaround, weil Eingang durch Wifi blockiert wird!
  WiFi.disconnect(); 
  esp_wifi_stop();   // Schaltet das Funkmodul aus
  delay(10);         // Kurze Pause zum Stabilisieren
  
  int raw = analogRead(UBatt_Pin);
  
  esp_wifi_start();  // Funk wieder an
  // Hier müsstest du ggf. ESP-NOW neu initialisieren
  return raw;
}

void EmergencyStop() {
  // 1. Initialisierung beim ersten Aufruf
  if (E_Stop == false) {
    E_Stop = true;
    HornTimer = millis();
    Horn = true;
    
    E_BlinkCount = 0;
    E_BlinkTimer = millis();
    E_BlinkState = true; 
    Serial.println("NOTSTOPP: Verbindung verloren!");
  }

  // 2. Sofort-Maßnahmen (Motor aus)
  _4QD_POT = 0;
  //_4QD_ON = false;

  // 3. Horn-Timer
  if (millis() - HornTimer > HornInterval) {
    Horn = false;
  }

  // 4. Blink-Logik
  if (E_BlinkCount < 10) { // 10 Wechsel für 5x Blinken
    if (millis() - E_BlinkTimer > E_BlinkInterval) {
      E_BlinkTimer = millis();
      E_BlinkState = !E_BlinkState;
      E_BlinkCount++;
    }
    // Hier weisen wir die Blink-Zustände aktiv zu
    Licht_Front_R = (E_BlinkState == true) ? HIGH : LOW;
    Licht_Rueck_R = (E_BlinkState == true) ? HIGH : LOW;
  } else {
    // Nach dem Blinken: Dauerlicht
    Licht_Front_R = HIGH;
    Licht_Rueck_R = HIGH;
  }

  // Frontlichter im Notstopp immer aus
  Licht_Front_F = LOW;
  Licht_Rueck_F = LOW;
}

void updateHardware() {
  if (E_Stop) {
    // Im Notstopp alles sicher abschalten
    //digitalWrite(_4QD_ON_Pin, LOW);
    analogWrite(_4QD_POT_Pin, 0);
    digitalWrite(Horn_Pin, Horn);
    // WICHTIG: Auch im E_Stop müssen die Lichter die Variablenwerte bekommen!
    digitalWrite(Licht_Rueck_F_Pin, Licht_Rueck_F);
    digitalWrite(Licht_Rueck_R_Pin, Licht_Rueck_R);
    digitalWrite(Licht_Front_F_Pin, Licht_Front_F);
    digitalWrite(Licht_Front_R_Pin, Licht_Front_R);
  } else {
    // Normalbetrieb
    digitalWrite(_4QD_ON_Pin, _4QD_ON);
    digitalWrite(_4QD_DIR_Pin, _4QD_DIR);
    analogWrite(_4QD_POT_Pin, _4QD_POT);
    
    digitalWrite(Licht_Front_F_Pin, Licht_Front_F);
    digitalWrite(Licht_Front_R_Pin, Licht_Front_R);
    digitalWrite(Licht_Rueck_F_Pin, Licht_Rueck_F);
    digitalWrite(Licht_Rueck_R_Pin, Licht_Rueck_R);
    
    digitalWrite(Horn_Pin, Horn);
  }
}


void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("Setup...");
  Wire.begin();
  // Initialize Output- Pins
  for (int i = 0; i < sizeof(outputPins) / sizeof(outputPins[0]); i++) {
    pinMode(outputPins[i], OUTPUT);
    delay(10);
    digitalWrite(outputPins[i], 0);
    delay(10);
  }
  pinMode(_4QD_POT_Pin, OUTPUT);
  analogWriteFrequency(25, 20000);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  //WiFi.channel() = 0;
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESP-NOW Initialized");
    esp_now_register_recv_cb(OnDataRecv); 
  } else {
    Serial.println("Error initializing ESP-NOW");
    ESP.restart();
  }  
  // Register peer
  esp_now_peer_info_t peerInfo = {}; // Clear the struct memory
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  Serial.println("Registrierung beendet...");
  delay(100); // Dem Serial-Buffer Zeit geben

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Fehler: Peer konnte nicht hinzugefügt werden");
    return;
  }
  
  Serial.println("Peer erfolgreich hinzugefügt!");
  delay(100);

  // WICHTIG: Prüfe hier, ob die Structs im Speicher okay sind
  size_t txSize = sizeof(TXData);
  size_t rxSize = sizeof(RXData);

  Serial.printf("DEBUG: TX struct size: %d Bytes\n", txSize);
  Serial.printf("DEBUG: RX struct size: %d Bytes\n", rxSize);
  
  Serial.println("Setup vollständig abgeschlossen.");
}

void loop() {
  // 1. Sicherheit: Timeout prüfen
  if (millis() - RXTimer > RXInterval) {
    EmergencyStop();
  }
  
  // 2. Daten verarbeiten, falls neue angekommen sind
  if (RXSuccess == true) {
    prepareMessage();
    getMessage();
  }

  // 3. WICHTIG: Die Hardware mit den aktuellen Werten füttern
  updateHardware();
}
