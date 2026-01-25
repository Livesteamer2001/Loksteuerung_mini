
/*
  Handsteuergerät Version 0.8
  Software für Loksteuerung über ESP-NOW
  Andreas Hauschild 2026
  Prozessor: LOLIN(WEMOS) D1 mini (clone)




*/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <U8g2lib.h>          // https://github.com/olikraus/u8g2
#include <Wire.h>
#include <RunningMedian.h>    // https://github.com/RobTillaart/RunningMedian/blob/master/examples/RunningMedian/RunningMedian.ino


// MAC-Adresse der Loksteuerung 
uint8_t broadcastAddress[] = {0x88, 0x57, 0x21, 0xBF, 0x56, 0x98};
// Eigene MAC-Adresse
//uint8_t broadcastAddress[] = {0xA4, 0xCF, 0x12, 0xC2, 0x3D, 0xB5};

// Pin-Definitionen
const int Licht_F_Pin = 12;   //
const int Licht_R_Pin = 13;
const int Dir_R_Pin = 16;
const int Horn_Pin = 15;
const int Dir_F_Pin = 14;
const int UBatt_Pin = 2;
const int Poti_Pin = A0;      //  2 - 1024

const int buttonPins[] = {
  Licht_F_Pin,
  Licht_R_Pin,
  Dir_R_Pin,
  Horn_Pin,
  Dir_F_Pin,
  UBatt_Pin
};

const int TX_id = 100;
const int RX_id = 100;
int t_Licht_F = 0;
int t_Licht_R = 0;
int last_Licht_F = 0;
int last_Licht_R = 0;
bool Licht_F_Flag = false;
bool Licht_R_Flag = false;
long timer_Licht_F;
long timer_Licht_R;
long switchTime = 500;      // Zeit, innerhalb der der Schalter nochmal betätigt werden muss für 2. Funktio
long timer_buttonCheck;
long TXTimer;
long TXInterval = 200;
unsigned long lastWifiUpdate = 0;
int myRSSI = 0;

typedef struct FST_message {
  uint8_t id = 100;           // Identifizierung, sicher ist sicher...
  uint8_t version = 1;
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
  uint8_t id;           // Identifizierung, sicher ist sicher...
  uint8_t version;
  int UBatt;        // Batteriespannung, 10 Bit
  int IMot1;        // Motorstrom 1, 10 Bit
  int IMot2;        // Motorstrom 2, 10 Bit
  int Speed;        // Fahrgeschwindigkeit
  int Temp;         // Motortemperatur
  int RSSI;         // Empfangsfeldstärke, gespiegelt weil esp32 das einfacher kann
} __attribute__((packed)) LST_Message;

FST_message TXData;
LST_message RXData;

RunningMedian samples = RunningMedian(10);

// Variable to store if sending data was successful
bool TXSuccess;
int TXFailCounter = 0;
bool speedLock = true; // Startet immer gesperrt (beim Einschalten)

// Displaytreiber
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C display(U8G2_R2);  // 0.91" OLED 128x32


class ToggleSwitch {
  private:
    const int PIN;
    const unsigned long WINDOW = 750; 
    const unsigned long DEBOUNCE = 50; 

    int state = 0; 
    bool lastPhysState = HIGH; 
    unsigned long lastEdgeTime = 0;
    unsigned long firstPressTime = 0; // Zeitstempel des allerersten Einschaltens
    int toggleCount = 0;

  public:
    ToggleSwitch(int pinNumber) : PIN(pinNumber) {}

    void begin() {
      pinMode(PIN, INPUT_PULLUP);
      lastPhysState = digitalRead(PIN);
    }

    void update() {
      bool currentPhysState = digitalRead(PIN);
      unsigned long now = millis();

      // 1. Flankenerkennung (Einschalten / Ausschalten)
      if (currentPhysState != lastPhysState) {
        if (now - lastEdgeTime > DEBOUNCE) {
          lastEdgeTime = now;
          
          if (currentPhysState == LOW) { // Schalter wird geschlossen (AN)
            if (toggleCount == 0) {
              firstPressTime = now; // Start des 1-Sekunden-Fensters
            }
            toggleCount++;
          }
          lastPhysState = currentPhysState;
        }
      }

      // 2. Auswertung der Logik
      if (currentPhysState == HIGH) {
        // Wenn der Schalter OFFEN ist:
        // Wir setzen nur zurück, wenn das Fenster abgelaufen ist ODER wir noch gar nicht angefangen haben
        if (toggleCount == 0 || (now - firstPressTime > WINDOW)) {
          state = 0;
          toggleCount = 0;
        }
      } 
      else {
        // Wenn der Schalter GESCHLOSSEN ist (AN):
        if (toggleCount == 1) {
          // Erstes Mal eingeschaltet: Warte ab, ob die Sekunde vergeht
          if (now - firstPressTime > WINDOW) {
            state = 1; // Dauerhaft an
          }
        } 
        else if (toggleCount >= 2) {
          // Innerhalb des Fensters wurde ein zweites Mal eingeschaltet
          state = 2; 
        }
      }
    }

    int getState() { return state; }
};

// --- HIER ERSTELLEN WIR DIE SCHALTER ---
// Instanzen für zwei Schalter
ToggleSwitch swLicht_F(Licht_F_Pin);
ToggleSwitch swLicht_R(Licht_R_Pin);

class DualPinSwitch {
  private:
    const int PIN_1; // Führt zu Zustand 1
    const int PIN_2; // Führt zu Zustand 2
    const unsigned long OFF_DELAY = 300; // Zeit, die man in der Mitte "verweilen" darf
    const unsigned long DEBOUNCE = 50;

    int lastReportedState = 0;
    bool lastPhys1 = HIGH;
    bool lastPhys2 = HIGH;
    unsigned long lastEdgeTime = 0;

  public:
    DualPinSwitch(int p1, int p2) : PIN_1(p1), PIN_2(p2) {}

    void begin() {
      pinMode(PIN_1, INPUT_PULLUP);
      pinMode(PIN_2, INPUT_PULLUP);
    }

    void update() {
      bool cur1 = digitalRead(PIN_1);
      bool cur2 = digitalRead(PIN_2);
      unsigned long now = millis();

      // Entprellte Flankenerkennung
      if (cur1 != lastPhys1 || cur2 != lastPhys2) {
        if (now - lastEdgeTime > DEBOUNCE) {
          lastEdgeTime = now;
          lastPhys1 = cur1;
          lastPhys2 = cur2;
        }
      }

      // ZUSTANDS-LOGIK
      if (cur1 == LOW) {
        // Schalter steht auf Position 1
        lastReportedState = 1;
      } 
      else if (cur2 == LOW) {
        // Schalter steht auf Position 2
        lastReportedState = 2;
      } 
      else {
        // Schalter ist in der Mitte (beide HIGH)
        // Nur wenn die Zeit abgelaufen ist, melden wir Zustand 0
        if (now - lastEdgeTime > OFF_DELAY) {
          lastReportedState = 0;
        }
      }
    }

    int getState() {
      return lastReportedState;
    }
};

// Schalter nutzt Dir_F_Pin und Dir_R_Pin
DualPinSwitch swDir(Dir_F_Pin, Dir_R_Pin);

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  //Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    //Serial.println("Delivery success");
    TXSuccess = true;
    TXFailCounter = 0;
  }
  else{
    //Serial.println("Delivery fail");
    TXSuccess = false;
    TXFailCounter++;
    if (TXFailCounter > 1000) TXFailCounter = 1000;
  }
}

// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&RXData, incomingData, sizeof(RXData));
  if (RXData.id != TXData.id) {

  }
  Serial.print("Bytes received: ");
  Serial.println(len);
  
}

void buttonCheck() {
  /*if (millis() - timer_buttonCheck > 2 * switchTime){
    timer_buttonCheck = millis();
  // display test
    int aread = analogRead(Poti_Pin);
    
    display.clearBuffer();					// clear the internal memory
    display.setFont(u8g2_font_open_iconic_arrow_2x_t);  // choose a suitable font at https://github.com/olikraus/u8g2/wiki/fntlistall
    display.setCursor(0, 15);
    display.drawGlyph(0, 15, 0x47);	// write something to the internal memory
    display.sendBuffer();					// transfer internal memory to the display
  }
    // buttons test
    String msg = "Licht_F: ";
    msg += digitalRead(Licht_F_Pin);
    msg += " | Licht_R: ";
    msg += digitalRead(Licht_R_Pin);
    msg += " | Dir_F: ";
    msg += digitalRead(Dir_F_Pin);
    msg += " | Dir_R: ";
    msg += digitalRead(Dir_R_Pin);
    msg += " | Horn: ";
    msg += digitalRead(Horn_Pin);
    msg += " | UBatt: ";
    msg += digitalRead(UBatt_Pin);
    msg += " | Poti: ";
    msg += analogRead(Poti_Pin);
    msg += " | swLicht_F.getState: ";
    msg += swLicht_F.getState();
    msg += " | swLicht_R.getState: ";
    msg += swLicht_R.getState();
    msg += " | swDir.getState: ";
    msg += swDir.getState();*/
	String msg = "Hauptsch.: ";
	msg += TXData.Hptsch_ON;
	msg += " | Richt.: ";
	msg += TXData.Richtung;
	msg += " | Licht_F: ";
	msg += TXData.Licht_F; 
	msg += " | Licht_R: ";
	msg += TXData.Licht_R; 
	msg += " | Horn: ";
	msg += TXData.Horn; 
	msg += " | Fahrs.: ";
	msg += TXData.Fahrschalter; 
  msg += " | Poti: ";
  msg += analogRead(Poti_Pin);
  msg += " | speedLock: ";
  msg += speedLock;
  msg += " | UBatt: ";
  msg += RXData.UBatt;
    Serial.println(msg);
  
}


void prepareMessage() {
  // 1. Verbindung prüfen: Wenn zu viele Pakete verloren gingen, Sperre rein
  // Der Wert 10 ist ein Puffer. Sobald die Verbindung weg ist, wird gesperrt.
  if (TXFailCounter > 10) {
    speedLock = true;
  }

  int aread = analogRead(Poti_Pin);

  if (swDir.getState() == 0) { 
    TXData.Hptsch_ON = false; 
    speedLock = true;
  } else {
    TXData.Hptsch_ON = true;
  }
  // Wenn gesperrt ist UND das Poti auf Null (bzw. < 10 wegen Rauschen) gedreht wird -> Entsperren
  if (speedLock == true && aread < 10) {
    speedLock = false;
  }
  
  TXData.Richtung = swDir.getState();
  TXData.Licht_F = swLicht_F.getState();
  TXData.Licht_R = swLicht_R.getState();
  TXData.Horn = !digitalRead(Horn_Pin);
  
  if (speedLock == true) {
    // Wenn gesperrt, immer 0 senden, egal wo das Poti steht
    TXData.Fahrschalter = 0;
  } else {
    // Wenn entsperrt, normalen Wert senden
    // Hinweis: constrain verhindert, dass map negative Werte liefert, falls aread < 2 ist
    TXData.Fahrschalter = map(constrain(aread, 2, 1024), 2, 1024, 0, 99);
  }
}

void TXRX() {
  if (millis() - TXTimer > TXInterval) {
      TXTimer = millis();
      esp_now_send(broadcastAddress, (uint8_t *) &TXData, sizeof(TXData));
      // Optionaler Debug:
       Serial.printf("Gesendet: %d Bytes\n", sizeof(TXData));
  }
  myDisplay();
}

void myDisplay() {
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_t0_16_tf);  // choose a suitable font

  // --- Richtungspfeile zeichnen ---
  switch (TXData.Richtung) {
    case 0:
      drawStatusCircle(9, 16, 8);
      drawStatusCircle(9, 16, 2);
      break;
    case 1:
      drawUpArrow(0, 0, 20, 30);
      break;
    case 2:
      drawDownArrow(0, 0, 20, 30);
      break;
    default:
      break;
  }

  // --- Funkstörung hat oberste Priorität ---
  if (TXFailCounter > 8) {
    //RXData.RSSI = 0;            
    //if (digitalRead(UBatt_Pin) == LOW) TXFailCounter = 0; // Reset Möglichkeit
    
    if (isBlinkPhase(800)) {
      display.setCursor(34, 16);
      display.print(" Funk-");
      display.setCursor(36, 29);
      display.print("Störung");
    } 
  } 
  else {
    // --- Verbindung OK: Entscheidung was angezeigt wird ---
    
    // 1. TEXT AUSGABE (Nullstellung hat Vorrang vor Akku-Text)
    display.setCursor(30, 15);
    
    if (speedLock) {
       // Wenn gesperrt -> WARNUNG
       if (isBlinkPhase(900)) {
          display.print("Poti -> 0 !"); 
       }
    } else {
       // Wenn nicht gesperrt -> Zeige an, welcher Akku gerade gemessen wird
       if (digitalRead(UBatt_Pin) == LOW) {
         display.print("Senderakku");
       } else {
         display.print("Lok-Akku");
       }
    }

    // 2. BATTERIE BALKEN (Unabhängig vom Text)
    if (digitalRead(UBatt_Pin) == LOW) {
      // Senderakku messen
      int reading = analogRead(Poti_Pin);
      samples.add(reading);
      int zustand = map(samples.getAverage(), 803, 955, 0, 100);
      drawBattery(35, 22, zustand);
    } else {
      // Lok-Akku aus Datenpaket nehmen
      int zustand;
      int safeUBatt = constrain(RXData.UBatt, 1410, 2955);
      if (safeUBatt > 1600) {
        zustand = map(safeUBatt, 2750, 2955, 0, 100);
      } else {
        zustand = map(safeUBatt, 1410, 1575, 0, 100);
      }
      

      drawBattery(35, 22, zustand);
    }
  }

  drawSignalStrength(104, 24, RXData.RSSI);
  display.sendBuffer();         // transfer internal memory to the display
}

void drawUpArrow(int x, int y, int w, int h) {
  // Parameter: (x1,y1, x2,y2, x3,y3) -> Spitze oben, links unten, rechts unten
  display.drawTriangle(x + w/2, y, x, y + h, x + w, y + h);
}

// Zeichnet einen Pfeil nach UNTEN (Spitze unten)
void drawDownArrow(int x, int y, int w, int h) {
  // Parameter: (x1,y1, x2,y2, x3,y3) -> links oben, rechts oben, Spitze unten
  display.drawTriangle(x, y, x + w, y, x + w/2, y + h);
}

// Zeichnet einen gefüllten Kreis (Disc)
void drawStatusCircle(int x, int y, int r) {
  // Parameter: (x_mitte, y_mitte, radius)
  display.drawCircle(x, y, r);
}

// Gibt 'true' zurück, wenn wir uns in der "An"-Phase des Blinkens befinden
bool isBlinkPhase(unsigned long interval) {
  return (millis() / interval) % 2 == 0;
}

void drawSignalStrength(int x, int y, int strength) {
  // strength sollte von 0 bis 4 gehen
  for (int i = 1; i <= 4; i++) {
    int barHeight = i * 3; // Jeder Balken wird 3 Pixel höher
    if (i <= strength) {
      // Balken gefüllt zeichnen
      display.drawBox(x + (i * 5), y - barHeight, 3, barHeight);
    } else {
      // Balken nur als Umriss (optional)
      display.drawFrame(x + (i * 5), y - barHeight, 3, barHeight);
    }
  }
}

void drawBattery(int x, int y, int percent) {
  int safePercent = constrain(percent, 0, 100);
  
  // 1. Rahmen und Nippel (bleiben immer sichtbar als Orientierung)
  display.drawFrame(x, y, 20, 10);
  display.drawBox(x + 20, y + 2, 2, 6);
  
  // 2. Berechnung der Balkenbreite
  int barWidth = map(safePercent, 0, 100, 0, 16);

  // 3. Blink-Zustand ermitteln
  bool isVisible = true;
  if (safePercent < 20) {
    // Wechselt alle 500ms zwischen true und false
    isVisible = (millis() / 500) % 2 == 0;
  }

  // 4. Balken und Text nur zeichnen, wenn isVisible true ist
  if (isVisible) {
    // Balken im Inneren
    if (barWidth > 0) {
      display.drawBox(x + 2, y + 2, barWidth, 6);
    }

    // Prozentzahl daneben
    display.setFont(u8g2_font_6x13_tr);
    display.setCursor(x + 35, y + 8);
    display.print(safePercent);
    display.print("%");
  }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("Setup...");
  Wire.begin();
  // Initialize Button Pins
  for (int i = 0; i < sizeof(buttonPins) / sizeof(buttonPins[0]); i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    delay(10);
  }
  pinMode(Poti_Pin, INPUT);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  int targetChannel = 0; 
  WiFi.begin("DUMMY_SSID", "DUMMY_PASS", targetChannel, NULL, false);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESPNow abgeschlossen");

  display.begin();
  display.enableUTF8Print();		// enable UTF8 support for the Arduino print()
  // Umschaltung Lichtschalter Front - Schluss
  swLicht_F.begin();
  swLicht_R.begin();
  swDir.begin();
  TXData.id = TX_id;
  RXData.id = RX_id;
  speedLock = true;     // Bocksprung verhindern
  /*delay(2000);
  Serial.print("Größe des TX-Pakets (FST_message): ");
  Serial.print(sizeof(TXData));
  Serial.println(" Bytes");

  Serial.print("Größe des RX-Pakets (LST_message): ");
  Serial.print(sizeof(RXData));
  Serial.println(" Bytes");*
}

void loop() {
  swDir.update();
  swLicht_F.update();
  swLicht_R.update();
  prepareMessage();
  TXRX();
  
  //buttonCheck();
}