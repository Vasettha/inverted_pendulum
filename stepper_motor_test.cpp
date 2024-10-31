#include <Arduino.h>

// Define the stepper and the pins it will use
#define STEP_PIN 26
#define DIR_PIN 25
#define ENABLE_PIN 33

void setup() {
  // Enable the motor
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); 
  digitalWrite(DIR_PIN, HIGH);

}

void loop() {
    // For our stepper motor, the lowest reliable delay I found is 400 microseconds at 1/4 microstepping.
    digitalWrite(DIR_PIN, HIGH);
    for (int i; i < 800; i++){
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(400); 
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(400);
    }
    digitalWrite(DIR_PIN, LOW);
    for (int i; i < 800; i++){
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(400); 
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(400);
    }
}
