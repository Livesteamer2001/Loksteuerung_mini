/*
  Handsteuergerät Version 0.9 RC1
  Software für Loksteuerung über ESP-NOW
  Andreas Hauschild 2026
  Prozessor: LOLIN(WEMOS) D1 mini (clone)




*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <RunningMedian.h>

// --- KONFIGURATION & ADRESSEN ---
uint8_t broadcastAddress[] = {0x88, 0x57, 0x21, 0xBF, 0x56, 0x98}; // MAC der Lok
// Eigene MAC-Adresse
//uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0xC2, 0x3D, 0xB5};

// Pin-Definitionen
const int Licht_F_Pin = 12;
const int Licht_R_Pin = 13;
const int Dir_R_Pin   = 16;
const int Horn_Pin    = 15;
const int Dir_F_Pin   = 14;
const int UBatt_Pin   = 2;  // Schalter für Anzeige Lok-/Senderakku
const int Poti_Pin    = A0; // Fahrpoti

const int buttonPins[] = {Licht_F_Pin, Licht_R_Pin, Dir_R_Pin, Horn_Pin, Dir_F_Pin, UBatt_Pin};

// --- DATENSTRUKTUREN (Müssen in Sender & Lok identisch sein) ---
typedef struct FST_message {
  uint8_t id = 100;
  uint8_t version = 1;
  uint8_t Fahrschalter = 0; // 0-99
  bool Hptsch_ON = false;
  uint8_t Richtung = 0;     // 0=N, 1=F, 2=R
  uint8_t Bremse = 0;
  uint8_t Licht_F = 0;      // 0=Aus, 1=Dauer, 2=Zusatz
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
  int UBatt = 0;   // Rohwert oder mV
  int IMot1 = 0;   // Motorstrom
  int IMot2 = 0;
  int Speed = 0;   // Echte Geschwindigkeit (optional)
  int Temp = 0;    // Motortemperatur
  int RSSI = 0;    // Signalstärke Rückkanal
} __attribute__((packed)) LST_Message;

FST_message TXData;
LST_message RXData;
RunningMedian samples = RunningMedian(10); // Glättung für Senderakku-Messung

// --- VARIABLEN FÜR TIMING & STATUS ---
long TXTimer;
long TXInterval = 200;    // Alle 200ms senden
int TXFailCounter = 0;    // Zählt verlorene Pakete
bool speedLock = true;    // Sicherheits-Sperre (Poti muss erst auf 0)

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R2);

// --- SCHALTER-LOGIK KLASSEN ---

// Verarbeitet einen Schalter (An/Aus) mit Doppelklick-Option
class ToggleSwitch {
  private:
    const int PIN;
    const unsigned long WINDOW = 750; // Zeitfenster für Doppelklick
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
          state = 0; toggleCount = 0;
        }
      } else {
        if (toggleCount == 1 && (now - firstPressTime > WINDOW)) state = 1;
        else if (toggleCount >= 2) state = 2; 
      }
    }
    int getState() { return state; }
};

// Verarbeitet einen 3-Wege-Schalter (z.B. Front-Aus-Rück)
class DualPinSwitch {
  private:
    const int PIN_1, PIN_2;
    const unsigned long OFF_DELAY = 300; // Toleranz für Mittelstellung
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
          lastEdgeTime = now; lastPhys1 = cur1; lastPhys2 = cur2;
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

// --- FUNK-FUNKTIONEN ---

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) TXFailCounter = 0; // Erfolg
  else {
    TXFailCounter++; // Fehler hochzählen
    if (TXFailCounter > 100) TXFailCounter = 100;
  }
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  if (len == sizeof(RXData)) {
    memcpy(&RXData, incomingData, sizeof(RXData));
  }
}

// --- HAUPTLOGIK & DISPLAY ---

void prepareMessage() {
  // Sicherheit: Bei Funkabriss Sperre aktivieren
  if (TXFailCounter > 10) speedLock = true;
  
  int aread = analogRead(Poti_Pin);

  // Hauptschalter: Nur AN, wenn Richtung gewählt (nicht Neutral)
  if (swDir.getState() == 0) { 
    TXData.Hptsch_ON = false; 
    speedLock = true;
  } else {
    TXData.Hptsch_ON = true;
  }

  // Entsperren: Wenn Poti auf 0 gedreht wurde
  if (speedLock && aread < 10) speedLock = false;
  
  // Daten füllen
  TXData.Richtung = swDir.getState();
  TXData.Licht_F = swLicht_F.getState();
  TXData.Licht_R = swLicht_R.getState();
  TXData.Horn = !digitalRead(Horn_Pin); // Pullup -> LOW wenn gedrückt
  
  // Fahrschalter-Wert berechnen
  TXData.Fahrschalter = speedLock ? 0 : map(constrain(aread, 2, 1024), 2, 1024, 0, 99);
}

void myDisplay() {
  display.clearBuffer();
  display.setFont(u8g2_font_t0_16_tf);

  // 1. Richtungspfeile
  switch (TXData.Richtung) {
    case 0: display.drawCircle(9, 16, 8); display.drawCircle(9, 16, 2); break; // Neutral
    case 1: display.drawTriangle(10, 0, 0, 30, 20, 30); break; // Vorwärts
    case 2: display.drawTriangle(0, 0, 20, 0, 10, 30); break;  // Rückwärts
  }

  // 2. Warnungen oder Akku-Status
  if (TXFailCounter > 8) {
    // Funkstörung blinkt
    if ((millis() / 800) % 2 == 0) {
      display.setCursor(34, 16); display.print(" Funk-");
      display.setCursor(36, 29); display.print("Störung");
    } 
  } else {
    display.setCursor(30, 15);
    if (speedLock) {
       // Poti-Warnung blinkt
       if ((millis() / 500) % 2 == 0) display.print("Poti -> 0 !"); 
    } else {
       // Welcher Akku wird gerade gemessen?
       display.print(digitalRead(UBatt_Pin) == LOW ? "Senderakku" : "Lok-Akku");
    }

    // Batteriebalken zeichnen
    int zustand;
    if (digitalRead(UBatt_Pin) == LOW) {
      // Intern: Sender-Batterie messen
      samples.add(analogRead(Poti_Pin)); 
      zustand = map(samples.getAverage(), 803, 955, 0, 100);
    } else {
      // Extern: Lok-Batterie aus Funkpaket (Beispiel-Mapping für 2S/4S)
      int safeUBatt = constrain(RXData.UBatt, 1410, 2955);
      zustand = (safeUBatt > 1600) ? map(safeUBatt, 2750, 2955, 0, 100) : map(safeUBatt, 1410, 1575, 0, 100);
    }
    drawBattery(35, 22, zustand);
  }

  // Signalstärke rechts oben
  for (int i = 1; i <= 4; i++) {
    int h = i * 3;
    if (i <= RXData.RSSI) display.drawBox(104 + (i * 5), 24 - h, 3, h);
    else display.drawFrame(104 + (i * 5), 24 - h, 3, h);
  }
  display.sendBuffer();
}

// Zeichnet Batterie-Icon und Text
void drawBattery(int x, int y, int percent) {
  int p = constrain(percent, 0, 100);
  display.drawFrame(x, y, 20, 10); // Gehäuse
  display.drawBox(x + 20, y + 2, 2, 6); // Nippel
  // Blinken bei unter 20%
  if (p >= 20 || (millis() / 500) % 2 == 0) {
    display.drawBox(x + 2, y + 2, map(p, 0, 100, 0, 16), 6);
    display.setFont(u8g2_font_6x13_tr);
    display.setCursor(x + 35, y + 8);
    display.print(p); display.print("%");
  }
}

// --- SETUP & LOOP ---

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 6; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) return;
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  esp_now_register_recv_cb(OnDataRecv);

  display.begin();
  display.enableUTF8Print();
  swLicht_F.begin();
  swLicht_R.begin();
  swDir.begin();
}

void loop() {
  // Eingänge aktualisieren
  swDir.update();
  swLicht_F.update();
  swLicht_R.update();
  
  // Datenpaket vorbereiten
  prepareMessage();
  
  // Senden in festem Intervall
  if (millis() - TXTimer > TXInterval) {
    TXTimer = millis();
    esp_now_send(broadcastAddress, (uint8_t *) &TXData, sizeof(TXData));
  }
  
  // Display auffrischen
  myDisplay();
}