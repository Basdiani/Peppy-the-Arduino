#include <DFPlayer_Mini_Mp3.h>
#include <stdlib.h>

// Pin-Definitionen
const int startButtonPin = 2;
const int buttonPins[] = {3, 4, 5, 6};
const int outputPins[] = {7, 8, 9, 10};
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

void initializePins() {
  // Serielle Kommunikation starten und DFPlayer Mini initialisieren
  Serial.begin(9600);
  if (!mp3.begin(Serial1)) {
    Serial.println("Fehler beim DFPlayer Mini");
    while (true);
  }

  mp3.volume(15); // Lautstärke einstellen

  // Pins konfigurieren
  pinMode(startButtonPin, INPUT);
  for (int i = 0; i < 4; ++i) {
    pinMode(buttonPins[i], INPUT);
    pinMode(outputPins[i], OUTPUT);
  }
  pinMode(partyLightPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  lastStartTime = millis(); // Startzeit setzen
}

// Funktion für den "Attract Mode"
void attractMode() {
  attractModeRunning = true;
  partyLightActive = true;
  digitalWrite(partyLightPin, HIGH);
  digitalWrite(outputPins[0], HIGH);
  digitalWrite(outputPins[1], HIGH);

  // Zufällig Track 2 oder Track 3 auswählen
  int attractTrack = random(2, 4); // Zufallszahl zwischen 2 (inklusive) und 4 (exklusive) (Track 2 und Track 3)

  mp3.play(attractTrack); // Zufällig ausgewählten Track abspielen

  delay(10000);
  digitalWrite(outputPins[0], LOW);
  digitalWrite(outputPins[1], LOW);
  attractModeRunning = false;
  partyLightActive = false;
  digitalWrite(partyLightPin, LOW);
  lastStartTime = millis();
  mp3.stop();
  digitalWrite(motorPin, LOW);
  digitalWrite(lightPin, LOW);
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

  mp3.play(1); // Track 1 abspielen, wenn das Spiel gestartet wird (falls 1 der Track für das Spiel ist)
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
  mp3.stop();
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
  if (digitalRead(startButtonPin) == HIGH && !programRunning) {
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
}
