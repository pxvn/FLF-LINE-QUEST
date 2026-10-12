// Add these includes at the top
#include <Arduino.h>
#include <HardwareSerial.h>
HardwareSerial Serial1(USART1);

// Pin definitions for IR sensors (verify these don't conflict with I2C)
const int irPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };
// Calibration storage
int sensorMin[7];
int sensorMax[7];
int sensorThreshold[7];

const int LED_PIN = PC13;

// Motor pins
const int motorA1 = PB15;
const int motorA2 = PB14;
const int motorPWMA = PA15;
const int motorB1 = PA8;
const int motorB2 = PB3;
const int motorPWMB = PB4;

// Motor speed constants
const int BASE_SPEED = 130;  // Reduce if currently unstable
const int MAX_SPEED = 255;   // Kept the same
const int MIN_SPEED = -255;  // Kept the same

// PID Constants adjusted to reduce oscillations
float Kp = 0.05;  // Increase slightly to make corrections faster
float Ki = 0.2;   // Reduce to minimize cumulative error
float Kd = 1;     // Increase to dampen oscillations better

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
int lastValidPosition = 2000;                 // Last known good line position
LineState currentState = LOST_LINE;

// Add these variables at the top with other globals
int lastLeftSpeed = 0;  // Store last known good speeds
int lastRightSpeed = 0;
float lastCorrection = 0;  // Store last PID correction
bool allWhite = false;     // Flag to track when all sensors see white

// Add these constants for search pattern
const int SEARCH_SPEED = 130;             // Speed for search movements
const int TURN_SPEED = 140;               // Speed for 180-degree turn
const unsigned long WAIT_TIME = 500;      // 1 second wait
const unsigned long FORWARD_TIME = 350;   // 1 second forward
const unsigned long TILT_TIME = 370;      // Time for each tilt
const unsigned long TURN_TIME = 330;      // Time for 180 degree turn
const unsigned long FINAL_FORWARD = 700;  // Forward after turn

// Add these variables with other globals
unsigned long searchStartTime = 0;
bool searchPatternStarted = false;
enum SearchPhase {
  INITIAL_STOP,
  TILT_LEFT,   // Changed order
  TILT_RIGHT,  // Changed order
  MOVE_FORWARD,
  TURN_AROUND,
  FINAL_FORWARD_MOVE
} searchPhase = INITIAL_STOP;

// Add these variables for non-blocking delays
unsigned long phaseStartTime = 0;
bool pauseInProgress = false;
const unsigned long PAUSE_DURATION = 500;  // 1 second pause between phases

// Add this global variable near the top with other globals
int currentSensorValues[7] = { 0, 0, 0, 0, 0, 0, 0 };  // Store current sensor readings

void setup() {
  analogReadResolution(12);
  for (int i = 0; i < 7; i++) pinMode(irPins[i], INPUT);

  // Motors
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorPWMA, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(motorPWMB, OUTPUT);
  stopMotors();

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial1.begin(115200);

  // Blink fast during calibration
  unsigned long startBlink = millis();
  while (millis() - startBlink < 500) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }

  // Run calibration
  calibrateSensors();

  // Turn ON after calibration
  digitalWrite(LED_PIN, HIGH);
}


void loop() {
  LineState newState = getLineState();
  float position = getLinePosition();

  // Add debug prints
  Serial1.print("Sensors: ");
  Serial1.print(getSensorString());
  Serial1.print(" State: ");
  Serial1.print(getStateName(newState));
  Serial1.println();  // New line

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
      }
      break;

    case LOST_LINE:
      if (millis() - lastLineTime <= LOST_LINE_TIMEOUT) {
        // During timeout, continue with last PID correction
        leftSpeed = BASE_SPEED + lastCorrection;
        rightSpeed = BASE_SPEED - lastCorrection;

        // Constrain speeds
        leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

        setMotorSpeeds(leftSpeed, rightSpeed);
        currentLeftSpeed = leftSpeed;
        currentRightSpeed = rightSpeed;
        if ((millis() / 500) % 2 == 0)
          digitalWrite(LED_PIN, HIGH);
        else
          digitalWrite(LED_PIN, LOW);

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
      digitalWrite(LED_PIN, HIGH);
      // Reset search pattern when line is found
      searchPatternStarted = false;
      // Normal PID line following
      float error;

      if (allWhite) {
        // If all sensors see white, use last correction
        error = lastError;  // Use last known error to maintain direction
      } else {
        // Normal line following when we see the line
        error = position - 2000;  // Change this to match new weight system
        lastError = error;        // Update last error only when we see the line
      }

      // PID calculation
      integral += error;
      integral = constrain(integral, -10000, 10000);
      float derivative = error - lastError;

      float correction = (Kp * error + Ki * integral + Kd * derivative) / 100;
      lastCorrection = correction;  // Save the correction for LOST_LINE state

      // Calculate motor speeds
      leftSpeed = BASE_SPEED + correction;
      rightSpeed = BASE_SPEED - correction;

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

void calibrateSensors() {
  Serial1.println("Starting 6s auto calibration... Moving clockwise");

  // Initialize min/max
  for (int i = 0; i < 7; i++) {
    sensorMin[i] = 4095;  // Max ADC for 12-bit
    sensorMax[i] = 0;
  }

  unsigned long startTime = millis();
  while (millis() - startTime < 6000) {
    // Drive in a slow clockwise circle
    setMotorSpeeds(-180, 180);  // Left motor slower, right motor faster

    for (int i = 0; i < 7; i++) {
      int val = analogRead(irPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }

    // Blink fast to indicate calibration in progress
    if ((millis() / 200) % 2 == 0)
      digitalWrite(LED_PIN, HIGH);
    else
      digitalWrite(LED_PIN, LOW);
  }

  stopMotors();  // Stop after calibration

  // Compute thresholds with +25% margin
  for (int i = 0; i < 7; i++) {
    int range = sensorMax[i] - sensorMin[i];
    // Place threshold 25% *above* black instead of near white
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
  }

  // Debug print
  Serial1.println("Calibration done:");
  for (int i = 0; i < 7; i++) {
    Serial1.print("S");
    Serial1.print(i);
    Serial1.print(": min=");
    Serial1.print(sensorMin[i]);
    Serial1.print(" max=");
    Serial1.print(sensorMax[i]);
    Serial1.print(" thr=");
    Serial1.println(sensorThreshold[i]);
  }

  // LED ON steady after calibration
  digitalWrite(LED_PIN, HIGH);
}



float getLinePosition() {
  float weights[7] = { 0, 1000, 2000, 3000, 4000, 5000, 6000 };
  float sum = 0;
  float weightedSum = 0;

  for (int i = 0; i < 7; i++) {
    int rawVal = analogRead(irPins[i]);
    currentSensorValues[i] = (rawVal < sensorThreshold[i]) ? 1 : 0;  // 1 = black line

    if (currentSensorValues[i] == 1) {
      weightedSum += 1000 * weights[i];
      sum += 1000;
    }
  }

  float position = (sum > 0) ? (weightedSum / sum) : 3000;  // Default center
  Serial1.print(" Pos:");
  Serial1.print(position);
  return position;
}



void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left motor (Motor A) - Swapped direction logic
  if (leftSpeed >= 0) {
    digitalWrite(motorA1, HIGH);  // Changed from HIGH
    digitalWrite(motorA2, LOW);   // Changed from LOW
  } else {
    digitalWrite(motorA1, LOW);   // Changed from LOW
    digitalWrite(motorA2, HIGH);  // Changed from HIGH
    leftSpeed = -leftSpeed;
  }

  // Right motor (Motor B) - Unchanged
  if (rightSpeed >= 0) {
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else {
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
    rightSpeed = -rightSpeed;
  }

  analogWrite(motorPWMA, leftSpeed);
  analogWrite(motorPWMB, rightSpeed);
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
  allWhite = true;  // assume no line until proven otherwise

  for (int i = 0; i < 7; i++) {
    if (currentSensorValues[i] == 1) {  // 1 = black line detected
      sensorsOnLine++;
      allWhite = false;  // at least one sensor sees black
      lastLineTime = millis();
    }
  }

  if (sensorsOnLine == 0) {
    if (millis() - lastLineTime > LOST_LINE_TIMEOUT) {
      return LOST_LINE;
    }
    return ON_LINE;  // temporarily assume still on line
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
  else return "-";                                  // Stopped
}

// Update getSensorString to use stored values and show white line as 'x'
String getSensorString() {
  char sensorStr[15];
  int idx = 0;
  for (int i = 0; i < 7; i++) {
    sensorStr[idx++] = (currentSensorValues[i] == 1) ? 'x' : '.';  // x = black
    if (i < 6) sensorStr[idx++] = ',';
  }
  sensorStr[idx] = '\0';
  return String(sensorStr);
}
