#include <arduinoFFT.h>

// BCD-Display Pins
int bcdA = 2;
int bcdB = 3;
int bcdC = 4;
int bcdD = 5;
int D3 = 6;  // Zehnerstelle
int D4 = 7;  // Einerstelle

// Potentiometer und LED Pins
int potPin = A5;
const int ledPin1 = A0;  // Pin für LED 1
const int ledPin2 = A1;  // Pin für LED 2
const int ledPin3 = A2;  // Pin für LED 3
const int buttonPin = 13;  // Pin für Taster

// FFT und LED-Matrix Pins
#define NUM_LEDS 8
#define DEMUX_DATA1 8
#define DEMUX_DATA2 9
#define DEMUX_DATA3 10
#define PNP_RED 11
#define PNP_GREEN 12

#define SAMPLES 64
#define SAMPLING_FREQUENCY 16000

ArduinoFFT<double> FFT_red = ArduinoFFT<double>();
ArduinoFFT<double> FFT_green = ArduinoFFT<double>();

// Zeitsteuerungen
const unsigned long sampleInterval = 500;
unsigned long previousMillis = 0;
const unsigned long multiplexInterval = 5000;
unsigned long lastMultiplexMicros = 0;
const unsigned long buttonDebounceInterval = 50;
unsigned long lastButtonPress = 0;
unsigned long fftPreviousMillis = 0;
const unsigned long fftInterval = 300;

unsigned int sampling_period_us;
double vReal_red[SAMPLES], vImag_red[SAMPLES];
double vReal_green[SAMPLES], vImag_green[SAMPLES];

// Variablen von LEDs und 7-Seg-Anzeige
int potentiometerValue, combinedRMS, displayValue;
int tens, units;
bool showingTens = true;
int buttonState = 0;       // Variable, um den aktuellen Buttonzustand zu speichern
int lastButtonState = 0;   // Variable, um den vorherigen Buttonzustand zu speichern
int currentLED = 0;        // Verfolgt, welche LED aktiv ist (0 = alle aus, 1 = LED 1, 2 = LED 2, 3 = LED 3)
int redLedCount = 0, greenLedCount = 0;

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

// Anzeige ausschalten 
void clearDisplay() {
  digitalWrite(D3, HIGH);
  digitalWrite(D4, HIGH);
}

// Aktivierung des Demultiplexers
void activateDemux(int index) {
  digitalWrite(DEMUX_DATA1, (index & 0x01) ? HIGH : LOW);
  digitalWrite(DEMUX_DATA2, (index & 0x02) ? HIGH : LOW);
  digitalWrite(DEMUX_DATA3, (index & 0x04) ? HIGH : LOW);
  delayMicroseconds(50);
}

// Variablen initialisieren und serielle Kommunikation starten
void setup() {
  pinMode(bcdA, OUTPUT);
  pinMode(bcdB, OUTPUT);
  pinMode(bcdC, OUTPUT);
  pinMode(bcdD, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  // die LED-Pins als Ausgänge setzen
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);  //Taster ist in Ruhezustand auf HIGH gesetzt
  
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

  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  clearDisplay();   // sicherstellt, dass die Anzeige leer ist
  Serial.begin(9600);  // aktiviert die serielle Kommunikation für Debugging
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();

  // Potentiometer-Wert abtasten
  if (currentMillis - previousMillis >= sampleInterval) {
    previousMillis = currentMillis;
    potentiometerValue = analogRead(potPin);  // ausliest den aktuellen Wert des Potentiometers 
    combinedRMS = potentiometerValue;
    displayValue = map(combinedRMS, 0, 1023, 0, 99);  // skaliert den Potentiometer-Wert auf einen Bereich von 0 bis 99
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

  // Tastersteuerung 
  int reading = digitalRead(buttonPin);
  if (reading == LOW && lastButtonState == HIGH && (currentMillis - lastButtonPress) > buttonDebounceInterval) {
    lastButtonPress = currentMillis;
    currentLED = (currentLED + 1) % 4;  // Zyklus durch 0, 1, 2, 3
  }
  lastButtonState = reading;

  // LED-Steuerung
  digitalWrite(ledPin1, (currentLED == 1) ? HIGH : LOW);
  digitalWrite(ledPin2, (currentLED == 2) ? HIGH : LOW);
  digitalWrite(ledPin3, (currentLED == 3) ? HIGH : LOW);

  // Signalverarbeitung mit FFT
  if (currentMillis - fftPreviousMillis >= fftInterval) {
    fftPreviousMillis = currentMillis;
 
  // Abtasten der Eingangssignale
  for (int i = 0; i < SAMPLES; i++) {
      unsigned long microseconds = micros();
      vReal_red[i] = analogRead(A3);
      vImag_red[i] = 0;
      vReal_green[i] = analogRead(A4);
      vImag_green[i] = 0;
      while (micros() < (microseconds + sampling_period_us)) {}
    }

    FFT_red.windowing(vReal_red, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_red.compute(vReal_red, vImag_red, SAMPLES, FFT_FORWARD);
    FFT_red.complexToMagnitude(vReal_red, vImag_red, SAMPLES);

    FFT_green.windowing(vReal_green, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT_green.compute(vReal_green, vImag_green, SAMPLES, FFT_FORWARD);
    FFT_green.complexToMagnitude(vReal_green, vImag_green, SAMPLES);
    
    // Berechnung der Magnitude in beiden Frequenzbereichen
    double highFreqMag = 0, lowFreqMag = 0;
    for (int i = 1; i < SAMPLES / 2; i++) {
      double frequency = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
      if (frequency > 2000 && frequency <= 4000) highFreqMag += vReal_red[i];
      if (frequency >= 0 && frequency <= 2000) lowFreqMag += vReal_green[i];
    }

    float maxRange = max(5000.0, max(highFreqMag, lowFreqMag) * 1.2);
    redLedCount = constrain(round(float(NUM_LEDS) * highFreqMag / maxRange), 0, NUM_LEDS);
    greenLedCount = constrain(round(float(NUM_LEDS) * lowFreqMag / maxRange), 0, NUM_LEDS);
  }

  // Aktualisierung der LED-Matrix 
  digitalWrite(PNP_RED, LOW);
  for (int i = 0; i < redLedCount; i++) {
    activateDemux(i);
    delayMicroseconds(500);
  }
  digitalWrite(PNP_RED, HIGH);
  delayMicroseconds(300);

  digitalWrite(PNP_GREEN, LOW);
  for (int i = 0; i < greenLedCount; i++) {
    activateDemux(i);
    delayMicroseconds(500);
  }
  digitalWrite(PNP_GREEN, HIGH);
  delayMicroseconds(300);
}
