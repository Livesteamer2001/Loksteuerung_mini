/*
  Handsteuergerät Version 0.9 RC3
  Software für Loksteuerung über ESP-NOW
  Andreas Hauschild 2026
  Prozessor: LOLIN(WEMOS) D1 mini (clone)
  Autopairing: Horn beim Einschalten > 10 sek drücken
*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <RunningMedian.h>
#include <EEPROM.h>

// --- KONFIGURATION & ADRESSEN ---
//uint8_t broadcastAddress[] = {0x88, 0x57, 0x21, 0xBF, 0x56, 0x98}; 

// Pin-Definitionen
const int Licht_F_Pin = 12;
const int Licht_R_Pin = 13;
const int Dir_R_Pin   = 16;
const int Horn_Pin    = 15;
const int Dir_F_Pin   = 14;
const int UBatt_Pin   = 2;  
const int Poti_Pin    = A0; 

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

FST_message TXData;
LST_message RXData;

// Struktur für Pairing-Nachricht (muss auf Sender und Empfänger gleich sein)
typedef struct Pairing_Message {
  uint8_t msgType; // 1 = Anfrage (Sender), 2 = Antwort (Lok)
  uint8_t macAddr[6];
} Pairing_Message;

// Variable für die Ziel-Adresse (wird aus EEPROM geladen)
uint8_t targetMAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
bool isPairingMode = false;

RunningMedian samples = RunningMedian(10); 

long TXTimer;
long TXInterval = 200; 
int TXFailCounter = 0; 
bool speedLock = true; 

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R0);

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
          state = 0; toggleCount = 0;
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

// --- FUNK ---
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) TXFailCounter = 0;
  else if (TXFailCounter < 100) TXFailCounter++;
}

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  if (len == sizeof(RXData)) memcpy(&RXData, incomingData, sizeof(RXData));
}

// Funktion zum Speichern der neuen MAC
void saveMacAddress(uint8_t *mac) {
  for (int i = 0; i < 6; i++) {
    EEPROM.write(i, mac[i]);
  }
  EEPROM.commit();
}

// Funktion zum Laden der MAC
void loadMacAddress() {
  uint8_t check = 0;
  for (int i = 0; i < 6; i++) {
    targetMAC[i] = EEPROM.read(i);
    if (targetMAC[i] == 0xFF) check++;
  }
  // Wenn EEPROM leer ist (alles FF), Standard-Broadcast lassen
  if (check == 6) Serial.println("Keine MAC gespeichert. Nutze Broadcast.");
}

void runPairingRoutine() {
  display.clearBuffer();
  display.setCursor(20, 20);
  display.print("PAIRING...");
  display.sendBuffer();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0) return;
  
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  
  // Temporäre Callback nur für Pairing
  esp_now_register_recv_cb([](uint8_t *mac, uint8_t *data, uint8_t len) {
    if (len == sizeof(Pairing_Message)) {
      Pairing_Message *msg = (Pairing_Message *)data;
      if (msg->msgType == 2) { // Antwort von Lok (Typ 2)
        saveMacAddress(mac); // MAC speichern
        
        display.clearBuffer();
        display.setCursor(10, 20);
        display.print("ERFOLG! Neustart");
        display.sendBuffer();
        delay(2000);
        ESP.restart();
      }
    }
  });

  // Broadcast Peer hinzufügen
  uint8_t broadcast[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_add_peer(broadcast, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  Pairing_Message req;
  req.msgType = 1; // Anfrage
  
  // Endlosschleife: Sendet alle 500ms eine Anfrage
  while (true) {
    esp_now_send(broadcast, (uint8_t *) &req, sizeof(req));
    
    // Kleines Blinken oder Animation im Display
    display.clearBuffer();
    display.setCursor(20, 15);
    display.print("Suche Lok...");
    if ((millis() / 500) % 2 == 0) display.drawDisc(64, 25, 3);
    display.sendBuffer();
    
    delay(500);
  }
}

// --- LOGIK ---
void prepareMessage() {
  if (TXFailCounter > 10) speedLock = true;
  bool messModusSender = (digitalRead(UBatt_Pin) == LOW);

  if (!messModusSender) {
    int aread = analogRead(Poti_Pin);
    if (speedLock && aread < 10) speedLock = false;
    if (!speedLock) {
      TXData.Fahrschalter = map(constrain(aread, 2, 1024), 2, 1024, 0, 99);
    } else {
      TXData.Fahrschalter = 0;
    }
  }

  TXData.Richtung = swDir.getState();
  if (TXData.Richtung == 0) { 
    TXData.Hptsch_ON = false; 
    TXData.Fahrschalter = 0; 
    speedLock = true;
  } else {
    TXData.Hptsch_ON = true;
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

  // 1. Richtungspfeile immer zeichnen
  switch (TXData.Richtung) {
    case 0: display.drawCircle(9, 16, 8); display.drawCircle(9, 16, 2); break;
    case 1: display.drawTriangle(10, 0, 0, 30, 20, 30); break;
    case 2: display.drawTriangle(0, 0, 20, 0, 10, 30); break;
  }

  // Hilfsvariablen
  bool messModusSender = (digitalRead(UBatt_Pin) == LOW);
  int zustand = 0;

  // 2. Haupt-Logik-Weiche
  if (messModusSender) {
    // --- PRIORITÄT 1: Sender-Akku Messung (Immer anzeigen!) ---
    display.setCursor(30, 15);
    display.print("Sender-Akku");
    
    samples.add(analogRead(Poti_Pin)); 
    zustand = map(samples.getAverage(), 803, 955, 0, 100);
    drawBattery(35, 22, zustand);
  } 
  else if (TXFailCounter > 8) {
    // --- PRIORITÄT 2: Funk-Störung (Nur wenn nicht Sender-Akku gemessen wird) ---
    if ((millis() / 800) % 2 == 0) {
      display.setCursor(34, 16); display.print(" Funk-");
      display.setCursor(36, 29); display.print("Störung");
    } 
  } 
  else {
    // --- PRIORITÄT 3: Normaler Betrieb (Lok-Akku & Poti-Warnung) ---
    bool potiStehtFalsch = (analogRead(Poti_Pin) >= 10);
    display.setCursor(30, 15);
    
    if (speedLock && potiStehtFalsch && (millis() / 500) % 2 == 0) {
       display.print("Poti -> 0 !"); 
    } else {
       display.print("Lok-Akku");
    }

    int safeUBatt = constrain(RXData.UBatt, 1410, 2955);
    zustand = (safeUBatt > 1600) ? map(safeUBatt, 2750, 2955, 0, 100) : map(safeUBatt, 1410, 1575, 0, 100);
    drawBattery(35, 22, zustand);
  }

  // 3. Signalstärke (RSSI) - Balken werden bei Störung leer gezeichnet
  for (int i = 1; i <= 4; i++) {
    int h = i * 3;
    // Wenn Verbindung OK und Balken-Index <= RSSI -> Ausgefüllt
    if (TXFailCounter <= 8 && i <= RXData.RSSI) {
      display.drawBox(104 + (i * 5), 24 - h, 3, h);
    } else {
      display.drawFrame(104 + (i * 5), 24 - h, 3, h);
    }
  }
  
  display.sendBuffer();
}

void setup() {
  Serial.begin(115200);
 
  // Pins initialisieren
  for (int i = 0; i < 6; i++) pinMode(buttonPins[i], INPUT_PULLUP);

  Serial.println("Setup");
  // Display starten
  display.begin();
  display.enableUTF8Print();
  display.clearBuffer();
  display.setFont(u8g2_font_6x13_tr);
  display.setCursor(10, 20);
  display.print("System Start...");
  display.sendBuffer();

  EEPROM.begin(64); // EEPROM starten

  
  // --- 1. PAIRING CHECK (Horn Taste > 10 Sek) ---
  //if (digitalRead(Horn_Pin) == LOW && digitalRead(UBatt_Pin) == LOW) {
  if (digitalRead(Horn_Pin) == LOW) {  
    unsigned long startPress = millis();
    bool enterPairing = false;
    
    // Schleife solange Taste gedrückt ist
    while (digitalRead(Horn_Pin) == LOW) {
      unsigned long pressedTime = millis() - startPress;
      
      // Visuelles Feedback
      display.clearBuffer();
      display.setFont(u8g2_font_6x13_tr);
      display.setCursor(10, 12);
      display.print("Pairing: halten");
      
      // Fortschrittsbalken zeichnen
      int progress = map(pressedTime, 0, 10000, 0, 108); // 108px Breite
      display.drawFrame(10, 18, 112, 10); // Rahmen
      display.drawBox(12, 20, progress, 6); // Füllung
      display.sendBuffer();

      // Wenn 5 Sekunden erreicht sind
      if (pressedTime >10000) {
        enterPairing = true;
        break; 
      }
      delay(50); // Kleines Delay zum Entprellen/Display schonen
    }

    if (enterPairing) {
      runPairingRoutine(); // Springt in die separate Pairing-Funktion (siehe unten)
    }
  }
  // ------------------------------------------------

  // Normaler Start weiter...
  loadMacAddress(); // Gespeicherte Lok-Adresse laden

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != 0) return;
  
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  esp_now_register_send_cb(OnDataSent);
  
  // Den gespeicherten Peer hinzufügen (WICHTIG: targetMAC nutzen!)
  esp_now_add_peer(targetMAC, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  
  esp_now_register_recv_cb(OnDataRecv);
  
  swLicht_F.begin();
  swLicht_R.begin();
  swDir.begin();
}

void loop() {
  swDir.update();
  swLicht_F.update();
  swLicht_R.update();
  prepareMessage();
  if (millis() - TXTimer > TXInterval) {
    TXTimer = millis();
    esp_now_send(targetMAC, (uint8_t *) &TXData, sizeof(TXData));
  }
  myDisplay();
}