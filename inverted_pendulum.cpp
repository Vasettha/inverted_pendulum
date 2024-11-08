#include <Arduino.h>
#include "AS5600.h"
#include <PID_v1.h>

// Pin definitions
#define STEP_PIN 26
#define DIR_PIN 25
#define ENABLE_PIN 33

// System parameters
#define MAX_PENDULUM_ERROR 45.0   // Maximum allowed angle deviation in degrees
#define MAX_MOTOR_STEPS 800       // Maximum allowed steps from home position
#define UPDATE_INTERVAL 5         // PID update interval in milliseconds
#define UPDATE_INTERVAL_MICROS (UPDATE_INTERVAL * 1000L)  // Convert to microseconds
#define STEPS_PER_REV 800        // Steps per revolution of your stepper motor
#define HOME_SPEED 2000          // Pulse delay for homing (microseconds)
#define VERTICAL_POSITION 878    // Raw encoder value at vertical position
#define MIN_STEP_TIME 600        // Minimum time for one complete step cycle (microseconds)
#define PULSE_WIDTH 400          // Width of the HIGH pulse in microseconds

AS5600L as5600;   // Use default Wire

double Setpoint = 0, Input, Output;
double Kp = 0.2, Ki = 9.0, Kd = 0.0;  
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// System state variables
long currentSteps = 0;
unsigned long lastUpdateTime = 0;

// Function to convert raw angle (0-4095) to degrees (-180 to +180)
float rawToDegrees(uint16_t rawAngle) {
    int16_t diff = rawAngle - VERTICAL_POSITION;
    
    if (diff > 2048) {
        diff -= 4096;
    } else if (diff < -2048) {
        diff += 4096;
    }
    
    return (diff * 360.0) / 4096.0;
}

bool checkSafetyLimits() {
    float currentAngle = rawToDegrees(as5600.readAngle());
    
    if (abs(currentAngle) > MAX_PENDULUM_ERROR || 
        abs(currentSteps) > MAX_MOTOR_STEPS) {
        return false;
    }
    return true;
}

void executeSteps(int numSteps) {
    if (numSteps == 0) return;
    
    unsigned long stepDelay = UPDATE_INTERVAL_MICROS / abs(numSteps);
    if (stepDelay < MIN_STEP_TIME) {
        stepDelay = MIN_STEP_TIME;
    }
    
    unsigned long startTime = micros();
    unsigned long nextStepTime = startTime;
    
    for (int i = 0; i < abs(numSteps); i++) {
        while (micros() < nextStepTime) {
            // Empty loop for precise timing
        }
        
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(PULSE_WIDTH);
        digitalWrite(STEP_PIN, LOW);
        
        nextStepTime = startTime + ((i + 1) * stepDelay);
    }
}

void setup() {
    Serial.begin(115200);

    // AS5600 initialization
    Wire.begin(21, 22);
    as5600.setSlowFilter(0x01);
    as5600.setAddress(0x36);
    as5600.begin(4);
    as5600.setDirection(AS5600_CLOCK_WISE);
    
    // Stepper initialization
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);  // Enable the driver

    // PID initialization
    int maxSteps = UPDATE_INTERVAL_MICROS / MIN_STEP_TIME;
    myPID.SetOutputLimits(-maxSteps, maxSteps);
    myPID.SetMode(AUTOMATIC);

    lastUpdateTime = micros();
}

void loop() {
    unsigned long currentTime = micros();
    
    if ((currentTime - lastUpdateTime) >= UPDATE_INTERVAL_MICROS) {
        lastUpdateTime = currentTime;
        
        if (!checkSafetyLimits()) {
            digitalWrite(ENABLE_PIN, HIGH);  // Disable motor
            while(1) {
                // Stop execution if safety limits are exceeded
                delay(1000);
            }
        }
        
        // Update PID with current angle in degrees
        Input = rawToDegrees(as5600.readAngle());
        myPID.Compute();
        
        // Round Output to nearest integer for step count
        int stepsToMove = floor(Output);
        
        if (stepsToMove != 0) {
            digitalWrite(DIR_PIN, stepsToMove >= 0 ? LOW : HIGH);
            executeSteps(abs(stepsToMove));
            currentSteps += stepsToMove;
        }
        
        // Optional: Send telemetry over Serial for monitoring
        unsigned long millisTime = millis();
        Serial.write((const char*)&millisTime, 4);
        float angle = rawToDegrees(as5600.readAngle());
        Serial.write((const char*)&angle, 4);
        Serial.write((const char*)&Output, 4);
        Serial.write((const char*)&currentSteps, 4);
    }
}
