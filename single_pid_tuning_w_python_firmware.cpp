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

// PID variables
double Setpoint, Input, Output;
double Kp = 1, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Add state enum
enum SystemState {
    WAITING_FOR_COMMAND,
    HOMING,
    RUNNING
};

// System state variables
long currentSteps = 0;
SystemState systemState = WAITING_FOR_COMMAND;
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

void homeMotor() {
    // Return to home position
    while (currentSteps != 0) {
        digitalWrite(DIR_PIN, currentSteps > 0 ? LOW : HIGH);
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(PULSE_WIDTH);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(HOME_SPEED - PULSE_WIDTH);
        currentSteps += (currentSteps > 0 ? -1 : 1);
    }
    // Send homing completion signal
    uint32_t homingComplete = 0xFFFFFFFF;
    Serial.write((const char*)&homingComplete, 4);
}

void stopSystem() {
    systemState = WAITING_FOR_COMMAND;
    Output = 0;
    homeMotor();
    digitalWrite(ENABLE_PIN, HIGH);
}

bool checkSafetyLimits() {
    float currentAngle = rawToDegrees(as5600.readAngle());
    
    if (abs(currentAngle) > MAX_PENDULUM_ERROR || 
        abs(currentSteps) > MAX_MOTOR_STEPS) {
        return false;
    }
    return true;
}

void startSequence() {
    digitalWrite(ENABLE_PIN, LOW);
    systemState = HOMING;
    lastUpdateTime = micros();
}

void executeSteps(int numSteps) {
    if (numSteps == 0) return;
    
    // Calculate delay between steps to distribute them evenly across the update interval
    unsigned long stepDelay = UPDATE_INTERVAL_MICROS / abs(numSteps);
    
    // Ensure we don't go below minimum step time
    if (stepDelay < MIN_STEP_TIME) {
        stepDelay = MIN_STEP_TIME;
    }
    
    unsigned long startTime = micros();
    unsigned long nextStepTime = startTime;
    
    for (int i = 0; i < abs(numSteps); i++) {
        // Wait until it's time for the next step
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
    // Limit maximum steps based on minimum step time
    int maxSteps = UPDATE_INTERVAL_MICROS / MIN_STEP_TIME;
    myPID.SetOutputLimits(-maxSteps, maxSteps);
    myPID.SetMode(AUTOMATIC);
}

void loop() {
    switch (systemState) {
        case WAITING_FOR_COMMAND:
            if (Serial.available() >= 12) {  // Expecting 3 float values (4 bytes each)
                float newKp, newKi, newKd;
                Serial.readBytes((char*)&newKp, 4);
                Serial.readBytes((char*)&newKi, 4);
                Serial.readBytes((char*)&newKd, 4);
                
                // Update PID parameters
                myPID.SetTunings(newKp, newKi, newKd);
                startSequence();
            }
            break;

        case HOMING:
            homeMotor();  // This will send completion signal when done
            systemState = RUNNING;
            break;

        case RUNNING:
            unsigned long currentTime = micros();
            
            if ((currentTime - lastUpdateTime) >= UPDATE_INTERVAL_MICROS) {
                lastUpdateTime = currentTime;
                
                if (!checkSafetyLimits()) {
                    stopSystem();
                    return;
                }
                
                // Update PID with current angle in degrees
                Input = rawToDegrees(as5600.readAngle());
                myPID.Compute();
                
                int stepsToMove = floor(Output);
                
                if (stepsToMove != 0) {
                    digitalWrite(DIR_PIN, stepsToMove >= 0 ? LOW : HIGH);
                    executeSteps(abs(stepsToMove));
                    currentSteps += stepsToMove;
                }
                
                // Send telemetry
                unsigned long millisTime = millis();
                Serial.write((const char*)&millisTime, 4);
                float angle = rawToDegrees(as5600.readAngle());
                Serial.write((const char*)&angle, 4);
                Serial.write((const char*)&Output, 4);
                Serial.write((const char*)&currentSteps, 4);
            }
            break;
    }
}
