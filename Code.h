#include "Arduino.h"
#include <DFRobotDFPlayerMini.h>
#include <stdlib.h>

// Pin-Definitionen
const int startButtonPin = 22;
const int buttonPins[] = {24, 4, 5, 6};
const int outputPins[] = {25, 8, 9, 10};
const int partyLightPin = 11;
const int motorPin = outputPins[0];
const int lightPin = outputPins[1];
const int ledPin = 13;

unsigned long startTime = 0;
unsigned long lastStartTime = 0;
bool programRunning = false;
bool attractModeRunning = false;
bool partyLightActive = false;

bool outputState[] = {false, false, false, false}; // Zustand der Ausgänge
#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/19, /*tx =*/18);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);


void setup() {
  #if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/19, /*tx =*/18);
#else
  FPSerial.begin(9600);
#endif

  Serial.begin(115200);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(10);  //Set volume value. From 0 to 30
  myDFPlayer.play(1);  //Play the first mp3

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  // Pins konfigurieren
  pinMode(startButtonPin, INPUT_PULLUP);
  for (int i = 0; i < 4; ++i) {
    pinMode(buttonPins[i], INPUT);
    pinMode(outputPins[i], OUTPUT);
  }
  pinMode(partyLightPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  lastStartTime = millis(); // Startzeit setzen

  //digitalWrite(startButtonPin, LOW);
}

// Funktion für den "Attract Mode"
void attractMode() {
  attractModeRunning = true;
  Serial.println(F("Attract mode gestartet"));
  partyLightActive = true;
  digitalWrite(partyLightPin, HIGH);
  digitalWrite(outputPins[0], HIGH);
  digitalWrite(outputPins[1], HIGH);

  // Zufällig Track 2 oder Track 3 auswählen
  int attractTrack = random(2, 4); // Zufallszahl zwischen 2 (inklusive) und 4 (exklusive) (Track 2 und Track 3)

  myDFPlayer.play(attractTrack); // Zufällig ausgewählten Track abspielen

  delay(10000);
  digitalWrite(outputPins[0], LOW);
  digitalWrite(outputPins[1], LOW);
  attractModeRunning = false;
  partyLightActive = false;
  digitalWrite(partyLightPin, LOW);
  lastStartTime = millis();
  myDFPlayer.stop();
  digitalWrite(motorPin, LOW);
  digitalWrite(lightPin, LOW);
  Serial.println(F("Attract mode Beendet"));
}

// Funktion, um das Spiel zu starten
void startGame() {
  programRunning = true;
  startTime = millis();
  attractModeRunning = false;
  partyLightActive = true;
  digitalWrite(partyLightPin, HIGH);
  digitalWrite(motorPin, HIGH);
  digitalWrite(lightPin, HIGH);

  myDFPlayer.play(1); // Track 1 abspielen, wenn das Spiel gestartet wird (falls 1 der Track für das Spiel ist)
   Serial.println(F("Game Gestartet"));
}

// Funktion, um das Spiel zu beenden
void endGame() {
  programRunning = false;
  digitalWrite(ledPin, LOW);
  lastStartTime = millis();
  partyLightActive = false;
  digitalWrite(partyLightPin, LOW);
  digitalWrite(motorPin, LOW);
  digitalWrite(lightPin, LOW);
  digitalWrite(outputPins[0], LOW);
  digitalWrite(outputPins[1], LOW);
  digitalWrite(outputPins[2], LOW);
  digitalWrite(outputPins[3], LOW);
  myDFPlayer.stop();
   Serial.println(F("Game Beendet."));
}

// Funktion zur Steuerung der Ausgänge basierend auf den Schaltern
void controlOutputs() {
  for (int i = 0; i < 4; ++i) {
    if (digitalRead(buttonPins[i]) == HIGH) {
      outputState[i] = true; // Schalter gedrückt, Output aktivieren
    } else {
      outputState[i] = false; // Schalter losgelassen, Output deaktivieren
    }
    digitalWrite(outputPins[i], outputState[i]);
  }
}

void loop() {
  unsigned long currentTime = millis();

  // Überprüfen, ob der "Attract Mode" gestartet werden soll
  if ((currentTime - lastStartTime) > 600000 && !attractModeRunning) {
    attractMode();
  }

  // Überprüfen, ob das Spiel gestartet werden soll
  if (digitalRead(startButtonPin) == LOW && !programRunning) {
    startGame();
  }

  // Wenn das Spiel läuft, steuere die Ausgänge und überprüfe die Spielzeit
  if (programRunning) {
    digitalWrite(ledPin, HIGH);
    controlOutputs();

    // Überprüfen, ob das Spielende erreicht wurde
    if (currentTime - startTime >= 60000) {
      endGame();
    }
  }
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
if (Serial.available() > 0) { // Prüfen, ob Daten in der seriellen Konsole verfügbar sind
    char command = Serial.read(); // Einlesen des ersten Zeichens

    // Überprüfen, welcher Befehl eingegeben wurde und entsprechende Aktion ausführen
    switch (command) {
      case 'A':
        // Führe Aktion für Befehl 'A' aus
        Serial.println("Befehl A erkannt!");
        // Hier kannst du den Code einfügen, der für Befehl 'A' ausgeführt werden soll
        attractMode();
        break;
      case 'B':
        // Führe Aktion für Befehl 'B' aus
        Serial.println("Befehl B erkannt!");
        // Hier kannst du den Code einfügen, der für Befehl 'B' ausgeführt werden soll
        endGame();
        break;
       case 'C':
        // Führe Aktion für Befehl 'B' aus
        Serial.println("Befehl B erkannt!");
        // Hier kannst du den Code einfügen, der für Befehl 'B' ausgeführt werden soll
        startGame();
         // Weitere Cases für weitere Befehle hinzufügen falls benötigt

      default:
        // Führe diese Aktion aus, wenn ein unbekannter Befehl eingegeben wurde
        Serial.println("Unbekannter Befehl!");
        break;
    }
}
}
void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
