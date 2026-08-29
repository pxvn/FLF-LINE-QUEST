// STM32 Line Following Robot - 8 Sensor Array
// Adapted from working ESP32 5-sensor code
// Enhanced for 8-sensor precision line following

#include <Arduino.h>

// Pin definitions for IR sensors (8 sensors)
const int irPins[8] = {A6, A5, A4, A3, A1, A0, A2, A7};

// Motor pins for STM32
const int motorA1 = PB5;      // AIN1
const int motorA2 = PB4;      // AIN2  
const int motorPWMA = PB3;    // PWMA
const int motorB1 = PB7;      // BIN1
const int motorB2 = PB8;      // BIN2
const int motorPWMB = PB9;    // PWMB

// Sensor threshold for STM32 12-bit ADC
const int SENSOR_THRESHOLD = 3850;

// Motor speed constants
const int BASE_SPEED = 200;    // Reduce if currently unstable
const int MAX_SPEED = 200;    // Kept the same
const int MIN_SPEED = -200;   // Kept the same

// PID Constants adjusted to reduce oscillations
float Kp = 6;    // Increase slightly to make corrections faster
float Ki = 1;    // Reduce to minimize cumulative error
float Kd = 15;    // Increase to dampen oscillations better

// PID Variables
float lastError = 0;
float integral = 0;

// Line following states
enum LineState {
  ON_LINE,
  LOST_LINE,
  FULL_STOP
};

// Additional variables for line handling
const unsigned long LOST_LINE_TIMEOUT = 200;  // Time to search for line before giving up
unsigned long lastLineTime = 0;               // Last time we saw the line
int lastValidPosition = 4000;                 // Last known good line position (center for 8 sensors)
LineState currentState = LOST_LINE;

// Add these variables at the top with other globals
int lastLeftSpeed = 0;  // Store last known good speeds
int lastRightSpeed = 0;
float lastCorrection = 0;  // Store last PID correction
bool allWhite = false;     // Flag to track when all sensors see white

// Add these constants for search pattern
const int SEARCH_SPEED = 130;               // Speed for search movements
const int TURN_SPEED = 140;                // Speed for 180-degree turn
const unsigned long WAIT_TIME = 500;      // 0.5 second wait
const unsigned long FORWARD_TIME = 350;   // 0.35 second forward
const unsigned long TILT_TIME = 370;       // Time for each tilt
const unsigned long TURN_TIME = 330;       // Time for 180 degree turn
const unsigned long FINAL_FORWARD = 700;  // Forward after turn

// Add these variables with other globals
unsigned long searchStartTime = 0;
bool searchPatternStarted = false;
enum SearchPhase {
  INITIAL_STOP,
  TILT_LEFT,      
  TILT_RIGHT,     
  MOVE_FORWARD,
  TURN_AROUND,
  FINAL_FORWARD_MOVE
} searchPhase = INITIAL_STOP;

// Add these variables for non-blocking delays
unsigned long phaseStartTime = 0;
bool pauseInProgress = false;
const unsigned long PAUSE_DURATION = 500; // 0.5 second pause between phases

// Add this global variable near the top with other globals
int currentSensorValues[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Store current sensor readings

void setup() {
  // Set ADC resolution for STM32
  analogReadResolution(12);
  
  // Initialize sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(irPins[i], INPUT);
  }

  // Initialize motor pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorPWMA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorPWMB, OUTPUT);

  // Stop motors initially
  stopMotors();

  
  // Motor test sequence
  motorTest();
}

void motorTest() {
  
  // 1. Test left motor forward (500ms)
  leftMotor(150);
  rightMotor(0);
  delay(500);
  
  // 2. Test right motor forward (500ms)
  leftMotor(0);
  rightMotor(150);
  delay(500);
  
  // 3. Test both motors forward (500ms)
  leftMotor(150);
  rightMotor(150);
  delay(500);
  
  // 4. Test left turn (sharp) - 500ms
  leftMotor(-150);
  rightMotor(150);
  delay(500);
  
  // 5. Test right turn (sharp) - 500ms
  leftMotor(150);
  rightMotor(-150);
  delay(500);
  
  // 6. Stop and prepare for line following
  stopMotors();
  delay(500);
}

void leftMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
  } else {
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    speed = -speed;
  }
  analogWrite(motorPWMA, constrain(speed, 0, 255));
}

void rightMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    speed = -speed;
  }
  analogWrite(motorPWMB, constrain(speed, 0, 255));
}

void loop() {
  LineState newState = getLineState();
  float position = getLinePosition();



  // Declare all speed variables at the start
  int leftSpeed = 0;
  int rightSpeed = 0;
  int currentLeftSpeed = 0;
  int currentRightSpeed = 0;

  // Update last valid position if we're on the line
  if (newState == ON_LINE) {
    lastValidPosition = position;
  }

  // Handle different line states
  switch (newState) {
    case FULL_STOP:
      stopMotors();
      while (1) {
        // Infinite loop - robot stops completely
      }
      break;

    case LOST_LINE:
      if (millis() - lastLineTime <= LOST_LINE_TIMEOUT) {
        // During timeout, continue with last PID correction
        leftSpeed = BASE_SPEED - lastCorrection;
        rightSpeed = BASE_SPEED + lastCorrection;

        // Constrain speeds
        leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

        setMotorSpeeds(leftSpeed, rightSpeed);
        currentLeftSpeed = leftSpeed;
        currentRightSpeed = rightSpeed;
      } else {
        // Start search pattern if not already started
        if (!searchPatternStarted) {
          searchStartTime = millis();
          phaseStartTime = millis();
          searchPatternStarted = true;
          searchPhase = INITIAL_STOP;
          pauseInProgress = false;
        }

        unsigned long phaseElapsedTime = millis() - phaseStartTime;

        // Execute search pattern
        switch (searchPhase) {
          case INITIAL_STOP:
            stopMotors();
            currentLeftSpeed = 0;
            currentRightSpeed = 0;
            if (phaseElapsedTime >= WAIT_TIME) {
              phaseStartTime = millis();
              searchPhase = TILT_RIGHT;
            }
            break;

          case TILT_LEFT:
            if (!pauseInProgress) {
              setMotorSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
              currentLeftSpeed = SEARCH_SPEED;
              currentRightSpeed = -SEARCH_SPEED;
              if (phaseElapsedTime >= TILT_TIME) {
                stopMotors();
                pauseInProgress = true;
                phaseStartTime = millis();
              }
            } else if (phaseElapsedTime >= PAUSE_DURATION) {
                pauseInProgress = false;
                phaseStartTime = millis();
                searchPhase = MOVE_FORWARD;
            }
            break;

          case TILT_RIGHT:
            if (!pauseInProgress) {
              setMotorSpeeds(-SEARCH_SPEED * 1.3, SEARCH_SPEED * 1.3);
              currentLeftSpeed = -SEARCH_SPEED * 1.3;
              currentRightSpeed = SEARCH_SPEED * 1.3;
              if (phaseElapsedTime >= TILT_TIME) {
                stopMotors();
                pauseInProgress = true;
                phaseStartTime = millis();
              }
            } else if (phaseElapsedTime >= PAUSE_DURATION) {
                pauseInProgress = false;
                phaseStartTime = millis();
                searchPhase = TILT_LEFT;
            }
            break;

          case MOVE_FORWARD:
            if (!pauseInProgress) {
              setMotorSpeeds(SEARCH_SPEED, SEARCH_SPEED);
              currentLeftSpeed = SEARCH_SPEED;
              currentRightSpeed = SEARCH_SPEED;
              if (phaseElapsedTime >= FORWARD_TIME) {
                stopMotors();
                pauseInProgress = true;
                phaseStartTime = millis();
              }
            } else if (phaseElapsedTime >= PAUSE_DURATION) {
                pauseInProgress = false;
                phaseStartTime = millis();
                searchPhase = TURN_AROUND;
            }
            break;

          case TURN_AROUND:
            if (!pauseInProgress) {
              setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
              currentLeftSpeed = -TURN_SPEED;
              currentRightSpeed = TURN_SPEED;
              if (phaseElapsedTime >= TURN_TIME) {
                stopMotors();
                pauseInProgress = true;
                phaseStartTime = millis();
              }
            } else if (phaseElapsedTime >= PAUSE_DURATION) {
                pauseInProgress = false;
                phaseStartTime = millis();
                searchPhase = FINAL_FORWARD_MOVE;
            }
            break;

          case FINAL_FORWARD_MOVE:
            setMotorSpeeds(SEARCH_SPEED, SEARCH_SPEED);
            currentLeftSpeed = SEARCH_SPEED;
            currentRightSpeed = SEARCH_SPEED;
            if (phaseElapsedTime >= FINAL_FORWARD) {
              // Reset search pattern to start over
              phaseStartTime = millis();
              searchPhase = INITIAL_STOP;
            }
            break;
        }
      }
      break;

    case ON_LINE:
      // Reset search pattern when line is found
      searchPatternStarted = false;
      // Normal PID line following
      float error;

      if (allWhite) {
        // If all sensors see white, use last correction
        error = lastError;  // Use last known error to maintain direction
      } else {
        // Normal line following when we see the line
        error = position - 4000;  // Center position for 8 sensors is 4000
        lastError = error;        // Update last error only when we see the line
      }

      // PID calculation
      integral += error;
      integral = constrain(integral, -10000, 10000);
      float derivative = error - lastError;

      float correction = (Kp * error + Ki * integral + Kd * derivative) / 100;
      lastCorrection = correction;  // Save the correction for LOST_LINE state

      // Calculate motor speeds
      leftSpeed = BASE_SPEED - correction;
      rightSpeed = BASE_SPEED + correction;

      // Constrain speeds
      leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
      rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

      // Apply motor speeds
      setMotorSpeeds(leftSpeed, rightSpeed);
      currentLeftSpeed = leftSpeed;
      currentRightSpeed = rightSpeed;
      break;
  }

  currentState = newState;
}

float getLinePosition() {
  // Position weights for 8 sensors (0-7000 range, center at 4000)
  float weights[8] = {7000, 6000, 5000, 4000, 3000, 2000, 1000, 0};
  float sum = 0;
  float weightedSum = 0;

  // Read sensor values using STM32 ADC
  for (int i = 0; i < 8; i++) {
    int analogValue = analogRead(irPins[i]);
    currentSensorValues[i] = (analogValue > SENSOR_THRESHOLD) ? 1 : 0; // 1 means black line detected
    
    // For line following, we want the black line
    int sensorValue = currentSensorValues[i] ? 1000 : 0;

    weightedSum += sensorValue * weights[i];
    sum += sensorValue;
  }

  float position = (sum > 0) ? weightedSum / sum : 4000; // Default to center
  return position;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor
  leftMotor(leftSpeed);
  // Right motor  
  rightMotor(rightSpeed);
}

void stopMotors() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, LOW);
  analogWrite(motorPWMA, 0);
  analogWrite(motorPWMB, 0);
}

LineState getLineState() {
  int sensorsOnLine = 0;
  allWhite = true;  // Initially assume all sensors see background

  // Use previously read sensor values
  for (int i = 0; i < 8; i++) {
    bool isOnBlack = (currentSensorValues[i] == 1);  // 1 means black line detected

    if (isOnBlack) {
      sensorsOnLine++;
      allWhite = false;         // At least one sensor sees black line
      lastLineTime = millis();  // Update last line time whenever we see the line
    }
  }

  // Regular line state detection
  if (sensorsOnLine == 0) {
    // Only return LOST_LINE if we haven't seen the line for LOST_LINE_TIMEOUT
    if (millis() - lastLineTime > LOST_LINE_TIMEOUT) {
      return LOST_LINE;
    }
    // Otherwise keep following the last known direction
    return ON_LINE;
  } else {
    return ON_LINE;
  }
}

String getStateName(LineState state) {
  switch (state) {
    case ON_LINE: return "ON_LINE";
    case LOST_LINE: return "LOST_LINE";
    case FULL_STOP: return "FULL_STOP";
    default: return "UNKNOWN";
  }
}

String getDirectionArrow(int leftSpeed, int rightSpeed) {
  if (leftSpeed > rightSpeed) return "<<<<<";       // Left turn
  else if (rightSpeed > leftSpeed) return ">>>>>";  // Right turn
  else if (leftSpeed > 0) return "^^^^^";           // Forward
  else if (leftSpeed < 0) return "vvvvv";           // Backward
  else return "-----";                              // Stopped
}

// Show sensor readings as binary string
String getSensorString() {
  String sensorStr = "";
  for (int i = 0; i < 8; i++) {
    sensorStr += (currentSensorValues[i] == 1) ? "1" : "0";
    if (i < 7) sensorStr += ",";
  }
  return sensorStr;
}
