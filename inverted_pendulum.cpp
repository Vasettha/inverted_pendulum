#include <Arduino.h>
#include "AS5600.h"
#include <PID_v1.h>
AS5600L as5600;   //  use default Wire

// Define the stepper and the pins it will use
#define STEP_PIN 26
#define DIR_PIN 25
#define ENABLE_PIN 33

double Setpoint, Input, Output;
double Kp=1, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

uint32_t pulseDelay =400;
unsigned long lastTime = millis();
unsigned long lastStepTime = micros();
int timeInterval = 10;
void setup()
{
    Serial.begin(115200);

    // AS5600 init
    Wire.begin(21,22,800000UL);
    as5600.setAddress(0x36);
    as5600.begin(4);  //  set direction pin.
    as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
    int b = as5600.isConnected();
    Serial.print("Connect: ");
    Serial.println(b);

    // Stepper init
    // Enable the motor
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW); // Enable the driver

    //PID init
    //initialize the variables we're linked to
    as5600.resetCumulativePosition();
    Input = as5600.getCumulativePosition();
    Setpoint = as5600.getCumulativePosition();

    myPID.SetOutputLimits(-1000, 1000);
    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    //timing
    lastTime = millis();
    lastStepTime = micros();
}


void loop()
{
    unsigned long now = millis();
    if (now - lastTime >= timeInterval)
    {
        lastTime = now;
    // Get the error
    Input = as5600.getCumulativePosition();
    Serial.println(Input);

    //Compute PID
    myPID.Compute();

    //Set direction based on Output
    if (Output >= 0){
        digitalWrite(DIR_PIN, HIGH);
    }
    else
    {
        digitalWrite(DIR_PIN,LOW);
    }
    }
    pulseDelay = map(abs(Output),0,1000,2000,400);
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(pulseDelay); // Short pulse to step the motor
    digitalWrite(STEP_PIN, LOW);
}


