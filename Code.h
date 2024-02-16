#include "Arduino.h"
#include <DFRobotDFPlayerMini.h>
#include <stdlib.h>
#include <avr/wdt.h> /* Header for watchdog timers in AVR */

// Pin-Definitionen
const int startButtonPin = 21; // PIN 21 für Interrupt bei Mega2560
const int buttonPins[] = {37, 39, 41, 43};
const int outputPins[] = {25, 27, 29, 31};
const int partyLightPin = 0;
const int motorPin = 24;
const int lightPin = 1;
const int ledPin = 13;
const int setattractpins[5][4] = {{HIGH,LOW,HIGH,HIGH},{LOW,HIGH,HIGH,HIGH},{HIGH,LOW,LOW,HIGH},{LOW,HIGH,HIGH,LOW},{HIGH,HIGH,HIGH,HIGH}};

unsigned long startTime = 0;
unsigned long lastStartTime = 0;
unsigned long MyTimer=0;
unsigned long MyAttractModeTimer=0;

bool programRunning = false;
bool attractModeRunning = false;
bool partyLightActive = false;

bool outputState[] = {true, true, true, true}; // Zustand der Ausgänge, true = high - relais low active
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
  Serial.println(F("Peppy wird konfiguriert..."));
  #if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/19, /*tx =*/18);
#else
  FPSerial.begin(9600);
#endif

  Serial.begin(115200);
Serial.println(F("Mit MP3 Player verbinden."));
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(500); // Code to compatible with ESP8266 watch dog.
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  
  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  // Pins konfigurieren
  pinMode(startButtonPin, INPUT_PULLUP);
  for (int i = 0; i < 4; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP);
    pinMode(outputPins[i], OUTPUT);
  }
  pinMode(partyLightPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(lightPin, OUTPUT);

  
  lastStartTime = millis(); // Startzeit setzen

  digitalWrite(ledPin, LOW);
  delay(200);
  
  pinreset(); // Standardwerte der Outputs setzen, player stoppen
  Serial.println(F("OutputPins auf Standard gesetzt."));

  wdt_enable(WDTO_8S);  /* Enable the watchdog with a timeout of 8 seconds */
  
  // Startbutton als Interrupt definieren
  attachInterrupt(digitalPinToInterrupt(startButtonPin), startbutton, FALLING);
  Serial.println(F("Interrupt für Gameplay aktiviert."));
}

// Funktion für den "Attract Mode"
void attractMode() {


  // setze Dauer des Attract Mode
MyTimer=millis();
MyAttractModeTimer=30000;
  
  attractModeRunning = true;
  Serial.println(F("Attract mode gestartet"));
  partyLightActive = true;

  // Zufällig Track 2 oder Track 3 auswählen
  int attractTrack = random(3, 5); // Zufallszahl zwischen 3 (inklusive) und 5 (exklusive) (Track 3 und Track 4)

  myDFPlayer.play(attractTrack); // Zufällig ausgewählten Track abspielen

  digitalWrite(partyLightPin, LOW);
  digitalWrite(motorPin, LOW);
  delay(200);

while(millis()-MyTimer<MyAttractModeTimer) {

  
// Peppy zappeln lassen
  for (int i = 0; i < 5; ++i) {
    pinset(setattractpins[i][0],setattractpins[i][1],setattractpins[i][2],setattractpins[i][3]);
    delay(750);
  }
  
  wdt_reset();  /* Reset the watchdog */
  }

  attractModeRunning = false;
  partyLightActive = false;
  myDFPlayer.stop();
  digitalWrite(partyLightPin, HIGH);
  digitalWrite(motorPin, HIGH);
  digitalWrite(lightPin, HIGH);
  delay(200);
  lastStartTime = millis();
  Serial.println(F("Attract mode Beendet"));
  loop(); // kein Return zum Case wenn per Command gestartet
}

// Funktion, um das Spiel zu starten
void startGame() {
  programRunning = true;
  attractModeRunning = false;
  partyLightActive = true;
  digitalWrite(ledPin, HIGH);
  digitalWrite(partyLightPin, LOW);
  digitalWrite(motorPin, LOW);
  digitalWrite(lightPin, LOW);
  delay(200);
  myDFPlayer.play(2); // Track 1 abspielen, wenn das Spiel gestartet wird (falls 1 der Track für das Spiel ist)
  startTime = millis();
  Serial.println(F("Game Gestartet"));
  loop(); // kein Return zum Case wenn per Command gestartet
}

// Funktion, um das Spiel zu beenden
void endGame() {
  
  digitalWrite(ledPin, LOW);
  delay(200);
  
//Pinoutputs auf standard
  pinreset();
  
  partyLightActive = false;
  programRunning = false;
  lastStartTime = millis();
  Serial.println(F("Game Beendet."));
  wdt_reset();  /* Reset the watchdog */

    // Startbutton als Interrupt wieder aktivieren
  attachInterrupt(digitalPinToInterrupt(startButtonPin), startbutton, FALLING);
  Serial.println(F("Imterrupt nach Gameende reaktiviert."));
  loop(); // kein Return zum Case wenn per Command gestartet
}

// Funktion zur Steuerung der Ausgänge basierend auf den Schaltern
bool controlOutputs() {
  for (int i = 0; i < 4; ++i) {
    if (digitalRead(buttonPins[i]) == LOW) {
      outputState[i] = false; // Schalter gedrückt, Output aktivieren
    } else {
      outputState[i] = true; // Schalter losgelassen, Output deaktivieren
    }
    digitalWrite(outputPins[i], outputState[i]);
  }
 delay(200);
  wdt_reset();  /* Reset the watchdog */
  return 1;
}

void loop() {
  
  // Wenn das Spiel läuft, steuere die Ausgänge und überprüfe die Spielzeit
  if (programRunning) { 

    //  Controlls aufrufen und Überprüfen, ob das Spielende erreicht wurde
    if (controlOutputs() && (millis() - startTime >= 60000)) {
      Serial.println(F("Spielende erreicht..."));
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
        Serial.println("Befehl C erkannt!");
        // Hier kannst du den Code einfügen, der für Befehl 'B' ausgeführt werden soll
        startGame();
         // Weitere Cases für weitere Befehle hinzufügen falls benötigt

      default:
        // Führe diese Aktion aus, wenn ein unbekannter Befehl eingegeben wurde
        Serial.println("Unbekannter Befehl!");
        break;
    }
}

// Überprüfen, ob der "Attract Mode" gestartet werden soll
  if ((millis() - lastStartTime) > 600000 && !attractModeRunning) {
    attractMode();
  }
wdt_reset();  /* Reset the watchdog */
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

void pinreset() {
  digitalWrite(partyLightPin, HIGH);
  digitalWrite(motorPin, HIGH);
  digitalWrite(lightPin, HIGH);
  digitalWrite(outputPins[0], HIGH);
  digitalWrite(outputPins[1], HIGH);
  digitalWrite(outputPins[2], HIGH);
  digitalWrite(outputPins[3], HIGH);
    myDFPlayer.stop();
    delay(200);
}

void pinset(int pina,int pinb,int pinc,int pind) {

  digitalWrite(outputPins[0], pina);
  digitalWrite(outputPins[1], pinb);
  digitalWrite(outputPins[2], pinc);
  digitalWrite(outputPins[3], pind);
  Serial.println(F("Peppy zappelt."));
  delay(250);  
}

void startbutton(){
    // Überprüfen, ob das Spiel gestartet werden soll
  if (!programRunning) {
        // Interrupt deaktivieren
    detachInterrupt(digitalPinToInterrupt(startButtonPin));
    Serial.println(F("Interrupt für Gameplay deaktiviert."));
    
    wdt_reset();  /* Reset the watchdog */
    
    //Sicherstellen, dass alle Pins auf standard zurückgesetzt sind (Attract Mode)
     pinreset();
  
    startGame();
  }
  }
