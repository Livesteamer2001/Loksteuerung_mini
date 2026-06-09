/*
  Programm für: LOLIN(WEMOS) D1 mini ESP32
  Handsteuergerät Version 1.1 (ESP32 Port)
  Software für Loksteuerung über ESP-NOW
  Andreas Hauschild 2026
  
  WICHTIGE ÄNDERUNGEN ZUM ESP8266:
  - Nutzt WiFi.h und esp_now.h (ESP32 API)
  - ADC-Auflösung: 12-Bit (0-4095 statt 0-1023)
  - Pin-Mapping auf GPIO-Nummern angepasst
  - Bremse geht auf Pin 39 (äußere Reihe)
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <RunningMedian.h>
#include <Preferences.h> // ESP32-Ersatz für EEPROM
#include <MovingAverage.h>

// --- KONFIGURATION & PIN-MAPPING (GPIO Nummern für ESP32 D1 Mini) ---
const int Licht_F_Pin = 19; // D6
const int Licht_R_Pin = 23; // D7
const int Dir_R_Pin   = 17; // D0
const int Horn_Pin    = 5;  // D8
const int Dir_F_Pin   = 18; // D5  Neu
const int UBatt_Pin   = 16; // D4
const int Poti_Pin    = 36; // A0 (VP / GPIO36)
const int Bremse_Pin  = 39; //  Neu

const int buttonPins[] = {Licht_F_Pin, Licht_R_Pin, Dir_R_Pin, Horn_Pin, Dir_F_Pin, UBatt_Pin};

// --- DATENSTRUKTUREN ---
typedef struct FST_message {
  uint8_t id = 100;
  uint8_t version = 1;
  uint8_t Fahrschalter = 0;
  bool Hptsch_ON = false;
  uint8_t Richtung = 0;     
  uint8_t Bremse = 0;
  uint8_t Licht_F = 0;      
  uint8_t Licht_R = 0;
  bool Licht_Fst = false;
  bool Panto_F = false;      
  bool Panto_R = false;
  bool Horn = false;
  bool Sound_ON = false;
  bool Sound_1 = false, Sound_2 = false, Sound_3 = false, Sound_4 = false;
} __attribute__((packed)) FST_message;

typedef struct LST_message {
  uint8_t id = 100;
  uint8_t version = 1;
  int UBatt = 0;
  int IMot1 = 0;   
  int IMot2 = 0;
  int Speed = 0;   
  int Temp = 0;    
  int RSSI = 0;
} __attribute__((packed)) LST_message;

const int Bremse_neutral = 50;
const int Bremse_loesen = 500;
const int Bremse_anlegen = 1750;
const int BremsDelta = 2;
// Messwerte für Batterieanzeige Loksteuergerät
const int UBatt_12V_Low = 1350;
const int UBatt_12V_High = 1515;
const int UBatt_24V_Low = 2875;
const int UBatt_24V_High = 3235;

FST_message TXData;
LST_message RXData;

typedef struct Pairing_Message {
  uint8_t msgType; // 1 = Anfrage, 2 = Antwort
  uint8_t macAddr[6];
} Pairing_Message;

uint8_t targetMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;
Preferences preferences;

RunningMedian samples = RunningMedian(64); 
MovingAverage <int16_t, 64> filter;

unsigned long TXTimer;
unsigned long TXInterval = 100; 
int TXFailCounter = 0;
bool speedLock = true; 
unsigned long lastPotiMillis = 0;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R2);

// UBatt Logik
unsigned long ubattPressStart = 0;
bool ubattWasPressed = false;
bool ubattMessModus = false;
unsigned long sound1Timer = 0;

// --- SCHALTER-KLASSEN ---
class ToggleSwitch {
  private:
    const int PIN;
    const unsigned long WINDOW = 750; 
    const unsigned long DEBOUNCE = 50; 
    int state = 0; 
    bool lastPhysState = HIGH;
    unsigned long lastEdgeTime = 0;
    unsigned long firstPressTime = 0;
    int toggleCount = 0;
  public:
    ToggleSwitch(int pinNumber) : PIN(pinNumber) {}
    void begin() { pinMode(PIN, INPUT_PULLUP); lastPhysState = digitalRead(PIN); }
    void update() {
      bool currentPhysState = digitalRead(PIN);
      unsigned long now = millis();
      if (currentPhysState != lastPhysState) {
        if (now - lastEdgeTime > DEBOUNCE) {
          lastEdgeTime = now;
          if (currentPhysState == LOW) {
            if (toggleCount == 0) firstPressTime = now;
            toggleCount++;
          }
          lastPhysState = currentPhysState;
        }
      }
      if (currentPhysState == HIGH) {
        if (toggleCount == 0 || (now - firstPressTime > WINDOW)) {
          state = 0;
          toggleCount = 0;
        }
      } else {
        if (toggleCount == 1 && (now - firstPressTime > WINDOW)) state = 1;
        else if (toggleCount >= 2) state = 2; 
      }
    }
    int getState() { return state; }
};

class DualPinSwitch {
  private:
    const int PIN_1, PIN_2;
    const unsigned long OFF_DELAY = 300;
    const unsigned long DEBOUNCE = 50;
    int lastReportedState = 0;
    bool lastPhys1 = HIGH, lastPhys2 = HIGH;
    unsigned long lastEdgeTime = 0;
  public:
    DualPinSwitch(int p1, int p2) : PIN_1(p1), PIN_2(p2) {}
    void begin() { pinMode(PIN_1, INPUT_PULLUP); pinMode(PIN_2, INPUT_PULLUP); }
    void update() {
      bool cur1 = digitalRead(PIN_1), cur2 = digitalRead(PIN_2);
      unsigned long now = millis();
      if (cur1 != lastPhys1 || cur2 != lastPhys2) {
        if (now - lastEdgeTime > DEBOUNCE) {
          lastEdgeTime = now;
          lastPhys1 = cur1; lastPhys2 = cur2;
        }
      }
      if (cur1 == LOW) lastReportedState = 1;
      else if (cur2 == LOW) lastReportedState = 2;
      else if (now - lastEdgeTime > OFF_DELAY) lastReportedState = 0;
    }
    int getState() { return lastReportedState; }
};

ToggleSwitch swLicht_F(Licht_F_Pin);
ToggleSwitch swLicht_R(Licht_R_Pin);
DualPinSwitch swDir(Dir_F_Pin, Dir_R_Pin);

// --- FUNK CALLBACKS (ESP32 API) ---
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) TXFailCounter = 0;
  else if (TXFailCounter < 100) TXFailCounter++;
}

/*void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  if (len == sizeof(RXData)) memcpy(&RXData, incomingData, sizeof(RXData));
}*/
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  // 1. Prüfen, ob es normale Fahrdaten (Telemetrie) sind
  if (len == sizeof(RXData)) {
    memcpy(&RXData, incomingData, sizeof(RXData));
  } 
  // 2. Prüfen, ob es eine Pairing-Antwort von einer Lok ist
  else if (len == sizeof(Pairing_Message)) {
    Pairing_Message *pm = (Pairing_Message*) incomingData;
    if (pm->msgType == 2) { // 2 = Antwort von der Lok
      saveMacAddress(pm->macAddr); // MAC im Flash-Speicher sichern
      
      // Kurze Rückmeldung im Display vor dem Neustart
      display.clearBuffer();
      display.setCursor(10, 20); 
      display.print("Lok gekoppelt!");
      display.sendBuffer();
      delay(2000);
      
      ESP.restart(); // ESP32 neu starten, damit er normal mit der neuen Lok bootet
    }
  }
}

// --- HILFSFUNKTIONEN ---
void saveMacAddress(const uint8_t *mac) {
  preferences.begin("loco-settings", false);
  preferences.putBytes("targetMAC", mac, 6);
  preferences.end();
}

void loadMacAddress() {
  preferences.begin("loco-settings", true);
  if (preferences.getBytesLength("targetMAC") == 6) {
    preferences.getBytes("targetMAC", targetMAC, 6);
  } else {
    Serial.println("Keine MAC gespeichert.");
  }
  preferences.end();
}

void runPairingRoutine() {
  display.clearBuffer();
  display.setCursor(20, 20); display.print("PAIRING..."); display.sendBuffer();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_recv_cb(OnDataRecv); // Callback für Antwort

  uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(peerInfo.peer_addr, broadcast, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  Pairing_Message req; req.msgType = 1;
  
  while (true) {
    esp_now_send(broadcast, (uint8_t *) &req, sizeof(req));
    display.clearBuffer();
    display.setCursor(20, 15); display.print("Suche Lok...");
    if ((millis() / 500) % 2 == 0) display.drawDisc(64, 25, 3);
    display.sendBuffer();
    
    // Check ob wir eine MAC empfangen haben (einfachste Logik: restart bei Erfolg)
    // In OnDataRecv müsste hier die Logik zum saveMacAddress eingebaut werden.
    delay(500);
  }
}

// --- HAUPTLOGIK ---
void updateUBatt() {
  bool currentState = (digitalRead(UBatt_Pin) == LOW);
  unsigned long now = millis();
  if (currentState && !ubattWasPressed) {
    ubattPressStart = now;
    ubattWasPressed = true;
    ubattMessModus = false;
  } 
  else if (currentState && ubattWasPressed) {
    if (now - ubattPressStart >= 500) ubattMessModus = true;
  } 
  else if (!currentState && ubattWasPressed) {
    if (!ubattMessModus) {
      TXData.Sound_1 = true;
      sound1Timer = now; 
    }
    ubattWasPressed = false;
    ubattMessModus = false;
  }

  if (TXData.Sound_1 && (now - sound1Timer > 300)) TXData.Sound_1 = false;
}

void prepareMessage() {
  if (TXFailCounter > 10) speedLock = true;
  TXData.Richtung = swDir.getState();

  // Poti-Logik alle 70ms [cite: 361]
  if (millis() - lastPotiMillis >= 70) {
    lastPotiMillis = millis();
    
    if (digitalRead(UBatt_Pin) == HIGH) {
      int aread = analogRead(Poti_Pin);
      filter.add(aread);

      // Sicherheits-Check gegen asynchrones Schalten [cite: 363, 365]
      // ESP32 ADC Max ist 4095, daher Schwelle angepasst (ca. 2800 für ESP32)
      if (aread > 2800) { 
        delay(10);
        if (digitalRead(UBatt_Pin) == LOW) return; 
      }

      if (speedLock && aread < 40) speedLock = false; 
      
      if (!speedLock) {
        // Mapping für 12-Bit ADC 
        TXData.Fahrschalter = map(constrain(filter.get(), 10, 4095), 10, 4095, 0, 99);
      } else {
        TXData.Fahrschalter = 0;
      }
    }
  }

  if (TXData.Richtung == 0) { 
    TXData.Fahrschalter = 0;
    speedLock = true;
    TXData.Hptsch_ON = false; 
    TXData.Sound_ON = false;
  } else {
    TXData.Hptsch_ON = true;
    TXData.Sound_ON = true;
  }
  
  TXData.Licht_F = swLicht_F.getState();
  TXData.Licht_R = swLicht_R.getState();
  TXData.Horn = !digitalRead(Horn_Pin);
}

void drawBattery(int x, int y, int percent) {
  int p = constrain(percent, 0, 100);
  display.drawFrame(x, y, 20, 10);
  display.drawBox(x + 20, y + 2, 2, 6);
  if (p >= 20 || (millis() / 500) % 2 == 0) {
    display.drawBox(x + 2, y + 2, map(p, 0, 100, 0, 16), 6);
    display.setFont(u8g2_font_6x13_tr);
    display.setCursor(x + 35, y + 8);
    display.print(p); display.print("%");
  }
}


void myDisplay() {
  display.clearBuffer();
  display.setFont(u8g2_font_t0_16_tf);
  switch (TXData.Richtung) {
    case 0: display.drawCircle(9, 16, 8); display.drawCircle(9, 16, 2); break;
    case 1: display.drawTriangle(10, 0, 0, 30, 20, 30); break;
    case 2: display.drawTriangle(0, 0, 20, 0, 10, 30); break;
  }

  if (ubattMessModus) {
    display.setCursor(30, 15); display.print("Sender-Akku");
    samples.add(analogRead(Poti_Pin));
    //Serial.println(samples.getAverage());
    // ACHTUNG: Kalibrierungswerte (3212, 3820) für ESP32 (12-Bit) grob geschätzt! 
    int zustand = map(samples.getAverage(), 3300, 4050, 0, 100);
    drawBattery(35, 22, zustand);
  } 
  else if (TXFailCounter > 8) {
    if ((millis() / 800) % 2 == 0) {
      display.setCursor(34, 16);
      display.print(" Funk-");
      display.setCursor(36, 29); display.print("Störung");
    } 
  } 
  else {
    bool potiStehtFalsch = (analogRead(Poti_Pin) >= 40);
    display.setCursor(30, 15);
    
    // --- NEU: Warnung bei Fahrtwunsch mit angezogener Bremse ---
    // Blinkende Version:
    if (TXData.Richtung > 0 && TXData.Fahrschalter > 10 && TXData.Bremse > 10) {
       if ((millis() / 500) % 2 == 0) {
          display.print("Bremse lösen!");
       }
    }
    else if (speedLock && potiStehtFalsch && (millis() / 500) % 2 == 0) {
       display.print("Poti -> 0 !");
    } else {
       display.print("Lok-Akku");
    }
    
    int safeUBatt = constrain(RXData.UBatt, 1300, 3300);
    int zustand = (safeUBatt > 1600) ? map(safeUBatt, UBatt_24V_High, UBatt_24V_Low, 0, 100) : map(safeUBatt, UBatt_12V_Low, UBatt_12V_High, 0, 100);
    drawBattery(35, 22, zustand);
  }

  for (int i = 1; i <= 4; i++) {
    int h = i * 3;
    if (TXFailCounter <= 8 && i <= RXData.RSSI) display.drawBox(104 + (i * 5), 24 - h, 3, h);
    else display.drawFrame(104 + (i * 5), 24 - h, 3, h);
  }
  display.sendBuffer();
}

void updateBremse() {
  int Bremswert = analogRead(Bremse_Pin);
  int Bremse = TXData.Bremse;
  if (Bremswert < Bremse_neutral) {
    // nichts tun
  } else if (Bremswert < Bremse_loesen){
    Bremse = Bremse - BremsDelta;
  } else {
    Bremse = Bremse + BremsDelta;
  }
  if (Bremse < 0) Bremse = 0;
  if (Bremse> 180) Bremse = 180;
  TXData.Bremse = Bremse;
}

void printData() {
  String msg = "Fahrsch: ";
  msg += TXData.Fahrschalter;
  msg += " | Hptsch: ";
  msg += TXData.Hptsch_ON;
  msg += " | Richt: ";
  msg += TXData.Richtung;
  msg += " | Bremse: ";
  msg += TXData.Bremse;
  msg += " | Licht_F: ";
  msg += TXData.Licht_F;
  msg += " | Licht_R: ";
  msg += TXData.Licht_R;
  msg += " | Horn: ";
  msg += TXData.Horn;
  msg += " | Snd_ON: ";
  msg += TXData.Sound_ON;
  msg += " | Snd_1:";
  msg += TXData.Sound_1;
  msg += " | Snd_2:";
  msg += TXData.Sound_2;
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  //Einschaltschutz / Totstell-Funktion via Horn-Pin ---
  // Wenn der Horn-Pin beim Einschalten NICHT gedrückt ist (also HIGH ist)
  if (digitalRead(Horn_Pin) == HIGH) { 
    // Endlosschleife: Das System tut nichts, das Display bleibt dunkel
    while (true) {
      delay(1000); 
    }
  }
  
  // Wenn der Horn-Pin gedrückt war (LOW), blockieren wir hier so lange,
  // bis der Pin wieder losgelassen wird (HIGH wird)
  while (digitalRead(Horn_Pin) == LOW) { 
    delay(20); // Kurze Pause zur Entlastung des Prozessors
  }
  // --- Ende Einschaltschutz ---

  display.begin();
  display.enableUTF8Print();
  display.clearBuffer();
  display.setFont(u8g2_font_6x13_tr);

  if (digitalRead(UBatt_Pin) == LOW) {
    runPairingRoutine();
  }

  display.setCursor(10, 20); display.print("System Start..."); display.sendBuffer();
  delay(1000);
  
  loadMacAddress();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, targetMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  swLicht_F.begin(); swLicht_R.begin(); swDir.begin();
}

void loop() {
  swDir.update();
  swLicht_F.update();
  swLicht_R.update();
  updateBremse();
  updateUBatt();
  prepareMessage();
  if (millis() - TXTimer > TXInterval) {
    TXTimer = millis();
    esp_now_send(targetMAC, (uint8_t *) &TXData, sizeof(TXData));
    //printData();
  }
  myDisplay();
}