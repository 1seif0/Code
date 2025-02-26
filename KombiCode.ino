#include <arduinoFFT.h>

////////////////////////////Initialisierung////////////////////////////////
//Taster
// Definition der Pins für LEDs und Taster
const int ledPin1 = A0;  // Pin für LED 1
const int ledPin2 = A1;  // Pin für LED 2
const int ledPin3 = A2;  // Pin für LED 3
const int buttonPin = 13; // Pin für Taster

// Definition der Variablen
int buttonState = 0;       // Variable, um den aktuellen Buttonzustand zu speichern
int lastButtonState = 0;   // Variable, um den vorherigen Buttonzustand zu speichern
int currentLED = 0;        // Verfolgt, welche LED aktiv ist (0 = alle aus, 1 = LED 1, 2 = LED 2, 3 = LED 3)

//Lichtorgel
#define NUM_LEDS 8           // Anzahl der LEDs pro Spalte
#define DEMUX_DATA1 8        // CD4514BE Data Pin A
#define DEMUX_DATA2 9        // CD4514BE Data Pin B
#define DEMUX_DATA3 10       // CD4514BE Data Pin C
#define PNP_RED 11           // PNP-Transistor für rote LEDs
#define PNP_GREEN 12         // PNP-Transistor für grüne LEDs
#define SAMPLES 64          // Anzahl der FFT-Samples
#define SAMPLING_FREQUENCY 16000  // Abtastfrequenz in Hz

ArduinoFFT<double> FFT_red = ArduinoFFT<double>();
ArduinoFFT<double> FFT_green = ArduinoFFT<double>();

unsigned long previousMillis = 0;  // Letzter Zeitpunkt der Abtastung
const unsigned long sampleInterval = 300; // Abtastintervall in Millisekunden
unsigned int sampling_period_us;

// Speicherung der FFT-Samples
double vReal_red[SAMPLES];
double vImag_red[SAMPLES];
double vReal_green[SAMPLES];
double vImag_green[SAMPLES];

// Variablen zur Steuerung der LEDs
int redLedCount = 0;
int greenLedCount = 0;

//7 Segmente
// Definition der BCD, der Anzeige und des Potentiometers-Pins
int bcdA = 2;  // BCD A Eingang
int bcdB = 3;  // BCD B Eingang
int bcdC = 4;  // BCD C Eingang
int bcdD = 5;  // BCD D Eingang
int D3 = 6;  // Zehnerstelle
int D4 = 7;  // Einerstelle
int potPin = A5;  

// RMS-Abtastintervall definieren
const unsigned long sampleInterval_7Seg = 500;  // Alle 0,5 Sekunden abtasten
unsigned long previousMillis_7Seg = 0;          // Zeitpunkt der letzten Abtastung

// Zeitsteuerung für das Multiplexen
const unsigned long multiplexInterval = 5000;
unsigned long lastMultiplexMicros = 0;       

// Variablen zum Speichern des RMS-Werte
int potentiometerValue, combinedRMS, displayValue;
int tens, units;
bool showingTens = true;
///////////////////Setup/////////////////////////////



void setup() {
  //Taster
 // die LED-Pins als Ausgänge setzen
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  //Taster ist in Ruhezustand auf HIGH gesetzt

  // aktiviert die serielle Kommunikation für Debugging
  //Serial.begin(9600);
  
  //Lichtorgel
  pinMode(DEMUX_DATA1, OUTPUT);
  pinMode(DEMUX_DATA2, OUTPUT);
  pinMode(DEMUX_DATA3, OUTPUT);
  pinMode(PNP_RED, OUTPUT);
  pinMode(PNP_GREEN, OUTPUT);

  digitalWrite(DEMUX_DATA1, LOW);
  digitalWrite(DEMUX_DATA2, LOW);
  digitalWrite(DEMUX_DATA3, LOW);
  digitalWrite(PNP_RED, HIGH);
  digitalWrite(PNP_GREEN, HIGH);

//Zeitabstand zwischen zwei Abtastungen, in Mikrosekunden, als Ganzzahl sichergestellt
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  //Serial.begin(9600); 

  //7 Segmente
// BCD-Pins als Ausgänge initialisieren
  pinMode(bcdA, OUTPUT);
  pinMode(bcdB, OUTPUT);
  pinMode(bcdC, OUTPUT);
  pinMode(bcdD, OUTPUT);
  
  // Ziffern-Pins als Ausgänge initialisieren
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);

   
  clearDisplay(); // sicherstellt, dass die Anzeige leer ist
}




//////////////////////Loop///////////////////////////////

void loop() {
  //Taster
// den Zustand des Tasters lesen
  buttonState = digitalRead(buttonPin);
  // überprüfet, ob sich der Tasterzustand geändert hat
  if (buttonState == LOW && lastButtonState == HIGH) { 
    // Der Taster wurde gedrückt, wechselt zur nächsten LED oder schalte alle aus
    currentLED = (currentLED + 1) % 4;  // Zyklus durch 0, 1, 2, 3
    //Serial.print("Wechsel zum Zustand ");
    //if (currentLED == 0) {
      //Serial.println("Alle LEDs AUS");
    //} else {
      //Serial.println(currentLED);
    }
  
  
  lastButtonState = buttonState;  // Aktualisierung des letzten Tasterzustands

  // alle LEDs am Anfang ausschalten
  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);
  digitalWrite(ledPin3, LOW);

 
  if (currentLED == 0) {
    // Alle LEDs aus 
    //Serial.println("Alle LEDs AUS");
  } else if (currentLED == 1) {
    // LED 1 wird eingeschaltet
    digitalWrite(ledPin1, HIGH);
    //Serial.println("LED 1 AN");
  } else if (currentLED == 2) {
    // LED 2 wird eingeschaltet
    digitalWrite(ledPin2, HIGH);
    //Serial.println("LED 2 AN");
  } else if (currentLED == 3) {
    // LED 3 wird eingeschaltet
    digitalWrite(ledPin3, HIGH);
    //Serial.println("LED 3 AN");
  }

  // Verzögerung, um die LED-Zeit zu simulieren
  delay(60);

//Lichtorgel
  // Aktuelle Zeit in Millisekunden abrufen
  unsigned long currentMillis = millis();
// Prüfen, ob es Zeit für eine neue Messung ist
  if (currentMillis - previousMillis >= sampleInterval) {
    previousMillis = currentMillis;// Letzte Messzeit aktualisieren

    // Signal ablesen
    for (int i = 0; i < SAMPLES; i++) {
      unsigned long microseconds = micros();
      //Spannung von Audiosignal ablesen (0-1023)
      vReal_red[i] = analogRead(A3); // Signal von A3 für hohe Frequenzen (rot)
      //vReal_red[i] = 512 + 100 * sin(2 * PI * 3000 * i / SAMPLING_FREQUENCY);
      vImag_red[i] = 0;
      vReal_green[i] = analogRead(A4); // Signal von A4 für niedrige Frequenzen (grün)
      //vReal_green[i] = 512 + 100 * sin(2 * PI * 1000 * i / SAMPLING_FREQUENCY);
      vImag_green[i] = 0;
       // Warten, um das richtige Zeitintervall zwischen Messungen einzuhalten
      while (micros() < (microseconds + sampling_period_us)) {
      }
    }
// FFT für das rote Signal berechnen
    FFT_red.windowing(vReal_red, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);// Hamming-Fenster anwenden，Spektralleckage reduzieren
    FFT_red.compute(vReal_red, vImag_red, SAMPLES, FFT_FORWARD);// FFT berechnen
    FFT_red.complexToMagnitude(vReal_red, vImag_red, SAMPLES); // Amplitude berechnen
 // FFT für das grüne Signal berechnen
    FFT_green.windowing(vReal_green, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_green.compute(vReal_green, vImag_green, SAMPLES, FFT_FORWARD);
    FFT_green.complexToMagnitude(vReal_green, vImag_green, SAMPLES);

// Speichert die Summe der Amplitude
    double highFrequencyMagnitude = 0;
    double lowFrequencyMagnitude = 0;

    for (int i = 1; i < SAMPLES / 2; i++) { // Nur die erste Hälfte der FFT-Daten nutzen (positive Frequenzen)
      double frequency = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
      if (frequency > 2000 && frequency <= 4000) {
        highFrequencyMagnitude += vReal_red[i];
      }
      if (frequency >= 0 && frequency <= 2000) {
        lowFrequencyMagnitude += vReal_green[i];
      }
    }

// Maximale Skalierung berechnen, um Amplitude zu LED Anzahl umzurechnen
// Mindestens 5000.0 oder 1.2-fache des höchsten Werts, um Übersteuerung zu vermeiden
  float maxRange = max(5000.0, max(highFrequencyMagnitude, lowFrequencyMagnitude) * 1.2); 
  redLedCount = constrain(round(float(NUM_LEDS) * highFrequencyMagnitude / maxRange), 0, NUM_LEDS);
  greenLedCount = constrain(round(float(NUM_LEDS) * lowFrequencyMagnitude / maxRange), 0, NUM_LEDS);


    //Serial.print("Rote LEDs (hohe Frequenz): ");
    //Serial.println(redLedCount);
    //Serial.print("Grüne LEDs (niedrige Frequenz): ");
    //Serial.println(greenLedCount);
  }

// Aktiviert die LED-Spalte (PNP-Transistor einschalten)
  digitalWrite(PNP_RED, LOW);
  for (int i = 0; i < redLedCount; i++) {
    activateDemux(i);
    delayMicroseconds(500); // Haltezeit für Stabilität，leuchtet genug lang
  }

  digitalWrite(PNP_RED, HIGH); // Spalte deaktivieren
  delayMicroseconds(300);      // warte bis alle LEDs ausschalten

  digitalWrite(PNP_GREEN, LOW);
  for (int i = 0; i < greenLedCount; i++) {
    activateDemux(i);
    delayMicroseconds(500); // Haltezeit für Stabilität
  }

  digitalWrite(PNP_GREEN, HIGH); // Spalte deaktivieren
  delayMicroseconds(300);        // warte bis alle LEDs ausschalten

  delayMicroseconds(800);

//7 Segmente
unsigned long currentMillis_7Seg = millis();
unsigned long currentMicros = micros();

  // Potentiometer-Wert abtasten
  if (currentMillis_7Seg - previousMillis_7Seg >= sampleInterval_7Seg) {
    previousMillis_7Seg = currentMillis_7Seg;
    potentiometerValue = analogRead(potPin);  // ausliest den aktuellen Wert des Potentiometers 
    displayValue = map(potentiometerValue, 0, 1023, 0, 99);   // skaliert den Potentiometer-Wert auf einen Bereich von 0 bis 99
    tens = displayValue / 10;  // berechnet der Zehnerstelle
    units = displayValue % 10;  // berechnet der Einerstelle
  }

  // Multiplexing zwischen den Ziffern der Anzeige
  if (currentMicros - lastMultiplexMicros >= multiplexInterval) {   // aktualisiert die Anzeige
    lastMultiplexMicros = currentMicros;   // speichert den aktuellen Zeitpunkt
    clearDisplay();
    if (showingTens) {
      displayDigit(tens, D3);
    } else {
      displayDigit(units, D4);
    }
    showingTens = !showingTens;
  }
}



/////////////Funktionen//////////////////
//Taster

//Lichtorgel
void activateDemux(int index) {
//alle Ausgänge vor der Änderung auf Null gesetzt werden
  digitalWrite(DEMUX_DATA1, LOW);
  digitalWrite(DEMUX_DATA2, LOW);
  digitalWrite(DEMUX_DATA3, LOW);
  delayMicroseconds(50);//Multiplexing

//Der Index wird in Binär konvertiert: 1 = High, 0 = Low, um den entsprechenden Kanal zu aktivieren
  digitalWrite(DEMUX_DATA1, (index & 0x01) ? HIGH : LOW);
  digitalWrite(DEMUX_DATA2, (index & 0x02) ? HIGH : LOW);
  digitalWrite(DEMUX_DATA3, (index & 0x04) ? HIGH : LOW);
  delayMicroseconds(50);//Multiplexing
}

//7 Segmente
// BCD-Anzeige steuern
void setBCDValue(int digit) {  // nimmt eine Zahl (0-9) und wandelt sie in ein 4-Bit-BCD um
  digitalWrite(bcdA, digit & 0x1);         // Bit0 (1)
  digitalWrite(bcdB, (digit >> 1) & 0x1);  // Bit1 (2)
  digitalWrite(bcdC, (digit >> 2) & 0x1);  // Bit2 (4)
  digitalWrite(bcdD, (digit >> 3) & 0x1);  // Bit3 (8)
}

void displayDigit(int digit, int digitPin) {
  setBCDValue(digit);   // vorbereitet die gewünschte Zahl auf der 7-Segment-Anzeige 
  delayMicroseconds(1);  // kleine Verzögerung um den Wert stabil zu setzen
  digitalWrite(digitPin, LOW);
}

// Funktion, um alle Ziffern auszuschalten
void clearDisplay() {
  digitalWrite(D3, HIGH); // Zehnerstelle deaktivieren
  digitalWrite(D4, HIGH); // Einerstelle deaktivieren
}
