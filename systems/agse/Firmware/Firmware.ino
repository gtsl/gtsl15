#include <MsTimer2.h>
#include <avr/sleep.h>

const int ledPin = 13; // Change this!
const int pauseInterrupt = 0; // This will equal pause pin on Arduino Due
const int pausePin = 2; // Change this!

boolean ledOut = false;

void setup() {
    pinMode(pausePin, INPUT);
    attachInterrupt(pauseInterrupt, sleepNow, HIGH); // If switch closed, call sleepNow

    pinMode(ledPin, OUTPUT);
    MsTimer2::set(1000, flash); // Flash led at 1 second
    MsTimer2::start();
}

void loop() {
    
}

void sleepNow() {
    digitalWrite(ledPin, HIGH);
    while (digitalRead(pausePin) != LOW) {;} // Only resume execution once pin goes low
}

void flash() {
    digitalWrite(ledPin, ledOut);
    ledOut = !ledOut;
}
