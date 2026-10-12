// Arduino Line Following Robot
// PID control with smart turn detection for 90-degree turns
// Enhanced edge sensor memory for sharp turn recovery
// Dynamic throttle system: starts at 140, increases gradually when on line, resets when line lost
// Adaptive PID: automatically adjusts PID gains based on current throttle for optimal control
// CURVED SENSOR ARRAY OPTIMIZATION - Enhanced for arc-shaped sensor layout

#include <Arduino.h>
#include <HardwareSerial.h>

// HardwareSerial instance for calibration output
HardwareSerial Serial1(USART1);

// ===== CURVED SENSOR ARRAY GEOMETRY =====
// 7-sensor array: L3, L2, L1, M0(center), R1, R2, R3
const uint8_t sensorPins[7] = { PA4, PA5, PA6, PA0, PA1, PA2, PA3 };

// Curved sensor array constants
#define CURVED_WEIGHT_MULTIPLIER 1.0
#define CURVED_ARRAY_CENTER 3000.0  // Center position for 7-sensor array

// Curved sensor array parameters (inward curve - sensors curve toward robot center)
// Distances and angles from center (M0) to each sensor
struct SensorGeometry {
  float distance;  // Distance from center in mm
  float angle;     // Angle from center in degrees (positive = left, negative = right)
  float weight;    // Position weight for line following
  float position;  // Calculated position value for this sensor
};

// Curved sensor geometry - corrected calculations
// Position values are calculated based on sensor spacing and curve geometry
const SensorGeometry sensorGeometry[7] = {
  // L3 (leftmost) - position 0
  { 46.5, 99.27, 6000, 0.0 },
  // L2 - position 1000
  { 23.019, 58.61, 5000, 1000.0 },
  // L1 - position 2000
  { 23.019, 40.66, 4000, 2000.0 },
  // M0 (center) - position 3000
  { 0.0, 0.0, 3000, 3000.0 },
  // R1 - position 4000
  { 21.76, -36.69, 2000, 4000.0 },
  // R2 - position 5000
  { 40.966, -72.07, 1000, 5000.0 },
  // R3 (rightmost) - position 6000
  { 64.0, -111.4, 0, 6000.0 }
};

// Dynamic threshold values - can be updated via web interface
int sensorThresholds[7] = { 2450, 2700, 3100, 3100, 3100, 3100, 2500 };

// Motor driver pins
#define PWMA PA15
#define AIN2 PB14
#define AIN1 PB15
#define BIN1 PA8
#define BIN2 PB3
#define PWMB PB4

// LED pin for calibration indication
#define LED_PIN PC13

// ===== TIMING CONSTANTS =====
#define SAMPLE_INTERVAL 1  // 1ms ultra-fast sensor reading
#define PID_INTERVAL 1     // 1ms ultra-fast PID for micro-corrections

// ===== PID VARIABLES =====
// Curved array optimized PID values - enhanced for better turn detection and precision
float baseKp = 0.06;  // Slightly higher response for curved array's better turn detection
float baseKi = 0.2;   // Slightly higher to eliminate offset with curved geometry
float baseKd = 0.02;  // Slightly higher damping for smoother curved path following

// PID values for maximum throttle (250) - optimized for curved array high-speed performance
float maxKp = baseKp;  // Higher proportional gain for curved array's enhanced precision
float maxKi = baseKi;  // Higher integral gain for curved array stability
float maxKd = baseKd;  // Higher derivative gain for curved array's smoother control

// Current adaptive PID values (will be calculated based on throttle)
float Kp, Ki, Kd;
int baseSpeed = 220;  // Starting base speed - will be managed by throttle

// ===== THROTTLE VARIABLES =====
int currentThrottle = 220;              // Current throttle level (starts at base)
int MIN_THROTTLE = 220;                 // Minimum throttle (base speed) - now updatable
int MAX_THROTTLE = 255;                 // Maximum throttle limit - now updatable
int THROTTLE_INCREMENT = 15;            // Speed increase per step - now updatable
unsigned long THROTTLE_INTERVAL = 150;  // 150ms between throttle increases - now updatable
unsigned long lastThrottleTime = 0;
unsigned long onLineStartTime = 0;
bool wasOnLine = false;

// ===== CONTROL PARAMETERS =====
int MAX_CORRECTION = 600;  // Increased for more responsive steering - now updatable
int ERROR_DEADBAND = 8;    // Reduced deadband for better responsiveness - now updatable

float error = 0, lastError = 0, integral = 0;
unsigned long lastPidTime = 0;
unsigned long lastSampleTime = 0;

// ===== LINE DETECTION VARIABLES =====
bool lineDetected = false;
unsigned long lineDetectedTime = 0;
int sampleCount = 0;

// ===== ENHANCED DOTTED LINE DETECTION VARIABLES =====
bool inDottedLineMode = false;                   // Flag to indicate if robot is in dotted line traversal mode
unsigned long dottedLineStartTime = 0;           // Timestamp when dotted line mode was entered
unsigned long lastLineDetectionTime = 0;         // Timestamp of the last successful line detection
int dottedLineForwardCount = 0;                  // Counter for how many forward steps taken in dotted line mode
const int MAX_DOTTED_FORWARD_STEPS = 10000;      // Increased for dotted line curves
const unsigned long DOTTED_LINE_TIMEOUT = 1200;  // Increased timeout for complex dotted line patterns

// Enhanced dotted line detection
bool lastDottedLinePattern = false;             // Track if we were in dotted line mode recently
unsigned long dottedLinePatternCount = 0;       // Count consecutive dotted line patterns
const int DOTTED_LINE_PATTERN_THRESHOLD = 3;    // Minimum patterns to confirm dotted line
const unsigned long PATTERN_RESET_TIME = 2000;  // Time to reset pattern counter

// Intersection detection
bool intersectionDetected = false;                    // Flag for T-junctions or intersections
unsigned long lastIntersectionTime = 0;               // When intersection was last detected
const unsigned long INTERSECTION_MEMORY_TIME = 1500;  // How long to remember intersection

// ===== SMART TURN VARIABLES =====
bool leftEdgeDetected = false;    // Sensor 0 detected black
bool rightEdgeDetected = false;   // Sensor 7 detected black
unsigned long leftEdgeTime = 0;   // When left edge was last detected
unsigned long rightEdgeTime = 0;  // When right edge was last detected
int lastTurnDirection = 0;        // -1 = left, 1 = right, 0 = straight
bool inRecoveryMode = false;      // Currently trying to recover line
unsigned long recoveryStartTime = 0;

int leftSpeed, rightSpeed, correction;
float currentPosition = 3000;  // Center position for 7-sensor array (sensor 3, PA0)


// ===== ROBOT CONTROL VARIABLES =====
volatile bool robotRunning = true;  // Default state is START

// ===== PID ADAPTATION FUNCTIONS =====
void updateAdaptivePID() {
  // Calculate throttle ratio (0.0 = MIN_THROTTLE, 1.0 = MAX_THROTTLE)
  float throttleRatio = (float)(currentThrottle - MIN_THROTTLE) / (MAX_THROTTLE - MIN_THROTTLE);

  // Linear interpolation between base and max PID values
  // At low throttle: use baseKp/Ki/Kd for aggressive corrections
  // At high throttle: use maxKp/Ki/Kd for smooth, stable control
  Kp = baseKp + (maxKp - baseKp) * throttleRatio;
  Ki = baseKi + (maxKi - baseKi) * throttleRatio;
  Kd = baseKd + (maxKd - baseKd) * throttleRatio;
}

int getAdaptiveDeadband() {
  // Increase deadband with throttle for smoother high-speed performance
  float throttleRatio = (float)(currentThrottle - MIN_THROTTLE) / (MAX_THROTTLE - MIN_THROTTLE);
  return ERROR_DEADBAND - (int)(throttleRatio * 35);  // 25-40 deadband range
}


// ===== CALIBRATION VARIABLES =====
#define CALIBRATION_DELAY 15            // Reduced delay for more samples during rotation
#define CALIBRATION_ROTATION_SPEED 150  // Rotation speed for smooth movement
bool calibrationComplete = false;
int sensorMinValues[7] = { 4095, 4095, 4095, 4095, 4095, 4095, 4095 };  // Start with max ADC value
int sensorMaxValues[7] = { 0, 0, 0, 0, 0, 0, 0 };                       // Start with min ADC value
int calibratedThresholds[7] = { 0, 0, 0, 0, 0, 0, 0 };                  // Will hold calibrated thresholds

// ===== CALIBRATION FUNCTIONS =====
void performSensorCalibration() {
  Serial1.println("=== STARTING SENSOR CALIBRATION ===");
  Serial1.println("Place robot on white surface (no black line)");
  Serial1.println("Robot will rotate clockwise for 6 seconds during calibration");
  Serial1.println("Calibrating in 3 seconds...");

  // Turn LED ON during calibration
  digitalWrite(LED_PIN, LOW);

  // Wait for user to place robot on white surface
  delay(1000);

  Serial1.println("Starting clockwise rotation...");

  // Start clockwise rotation and keep it going for 3 seconds
  leftMotor(CALIBRATION_ROTATION_SPEED);
  rightMotor(-CALIBRATION_ROTATION_SPEED);

  // Take samples while rotating for exactly 3 seconds
  unsigned long startTime = millis();
  unsigned long sampleCount = 0;

  while (millis() - startTime < 6000) {  // Rotate for exactly 6 seconds
    // Read all sensors
    for (int sensor = 0; sensor < 7; sensor++) {
      int value = analogRead(sensorPins[sensor]);

      // Update min and max values for each sensor
      if (value < sensorMinValues[sensor]) {
        sensorMinValues[sensor] = value;
      }
      if (value > sensorMaxValues[sensor]) {
        sensorMaxValues[sensor] = value;
      }
    }

    sampleCount++;

    // Keep rotating - no direction changes, just continuous clockwise rotation
    delay(CALIBRATION_DELAY);
  }

  // Stop motors after calibration
  stopMotors();
  Serial1.println("Calibration rotation complete, stopping motors...");

  // Calculate calibrated thresholds (middle value between min and max)
  Serial1.println("Calculating calibrated thresholds...");
  for (int sensor = 0; sensor < 7; sensor++) {
    // Calculate threshold as middle value between min and max
    // calibratedThresholds[sensor] = (sensorMinValues[sensor] + sensorMaxValues[sensor]) / 2;

    // Threshold at 30% between black (max) and white (min), closer to black
    calibratedThresholds[sensor] = sensorMinValues[sensor] + (sensorMaxValues[sensor] - sensorMinValues[sensor]) * 0.2;

    // Print calibration results for each sensor
    Serial1.print("Sensor ");
    Serial1.print(sensor);
    Serial1.print(": Min=");
    Serial1.print(sensorMinValues[sensor]);
    Serial1.print(", Max=");
    Serial1.print(sensorMaxValues[sensor]);
    Serial1.print(", Threshold=");
    Serial1.println(calibratedThresholds[sensor]);
  }

  // Update the global sensor thresholds with calibrated values
  for (int i = 0; i < 7; i++) {
    sensorThresholds[i] = calibratedThresholds[i];
  }



  calibrationComplete = true;
  Serial1.println("=== CALIBRATION COMPLETE ===");
  Serial1.println("Place robot on black line to test...");
  Serial1.println("Waiting 3 seconds before starting line following...");

  // Turn LED OFF after calibration is complete
  digitalWrite(LED_PIN, HIGH);

  delay(1000);
}

void displayCalibrationStatus() {
  if (calibrationComplete) {
    Serial1.println("=== CALIBRATION STATUS ===");
    for (int i = 0; i < 7; i++) {
      Serial1.print("Sensor ");
      Serial1.print(i);
      Serial1.print(": Threshold=");
      Serial1.print(sensorThresholds[i]);
      Serial1.print(" (Min=");
      Serial1.print(sensorMinValues[i]);
      Serial1.print(", Max=");
      Serial1.print(sensorMaxValues[i]);
      Serial1.println(")");
    }
    Serial1.println("========================");
  } else {
    Serial1.println("Calibration not yet performed!");
  }
}

// ===== CURVED SENSOR ARRAY READING FUNCTIONS =====
float readLinePosition() {
  float weightedSum = 0;
  float totalWeight = 0;
  int blackCount = 0;
  bool sensorsDetected[7] = { false, false, false, false, false, false, false };

  // Read all 7 sensors once
  for (int i = 0; i < 7; i++) {
    int value = analogRead(sensorPins[i]);

    // Use fixed threshold for each sensor
    // Lower values indicate black line (we follow black)
    if (value < sensorThresholds[i]) {  // black line detected
      sensorsDetected[i] = true;
      blackCount++;

      // Apply curved array weighting based on sensor geometry
      float sensorWeight = sensorGeometry[i].weight;


      // Use the sensor's position value for weighted calculation
      weightedSum += sensorGeometry[i].position * sensorWeight;
      totalWeight += sensorWeight;

      // Enhanced edge sensor detection for curved array
      if (i == 0 || i == 1) {  // Leftmost sensors (L3, L2) - more sensitive due to curve
        leftEdgeDetected = true;
        leftEdgeTime = millis();
      }
      if (i == 5 || i == 6) {  // Rightmost sensors (R2, R3) - more sensitive due to curve
        rightEdgeDetected = true;
        rightEdgeTime = millis();
      }

      // Enhanced intersection detection for curved array
      // Check for multiple sensors detecting line simultaneously (potential intersection)
      if (blackCount >= 3) {
        // Multiple sensors active - could be intersection or wide line
        if (blackCount >= 4) {
          intersectionDetected = true;
          lastIntersectionTime = millis();
        }
      }
    }
  }

  // Return position if we detected black line, otherwise return -1 to indicate no line
  if (blackCount > 0) {
    // Calculate weighted average position for curved array
    float averagePosition = weightedSum / totalWeight;

    // Apply curved array correction factor based on detected sensors
    float correctionFactor = 1.0;

    // If outer sensors are detected, apply curve correction
    if (sensorsDetected[0] || sensorsDetected[6]) {         // L3 or R3 detected
      correctionFactor = 1.15;                              // Boost position calculation for outer curve
    } else if (sensorsDetected[1] || sensorsDetected[5]) {  // L2 or R2 detected
      correctionFactor = 1.08;                              // Moderate boost for inner curve
    }

    return averagePosition * correctionFactor;
  } else {
    return -1;  // No line detected
  }
}

void stopMotors() {
  leftMotor(0);
  rightMotor(0);
}

void sharpLeftTurn() {
  leftMotor(-200);  // Maximum reverse left motor
  rightMotor(200);  // Maximum forward right motor
}

void sharpRightTurn() {
  leftMotor(200);    // Maximum forward left motor
  rightMotor(-200);  // Maximum reverse right motor
}

void fastLeftSearch() {
  leftMotor(50);    // Slow forward left
  rightMotor(200);  // Fast forward right
}

void fastRightSearch() {
  leftMotor(200);  // Fast forward left
  rightMotor(50);  // Slow forward right
}



// ===== MOTOR FUNCTIONS =====
void leftMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMA, constrain(speed, 0, 255));
}

void rightMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    speed = -speed;
  }
  analogWrite(PWMB, constrain(speed, 0, 255));
}

// ===== MAIN LINE FOLLOWING LOGIC =====
void executeLineFollowing() {
  // If robot is stopped, just stop motors and return
  if (!robotRunning) {
    stopMotors();
    return;
  }

  // Ensure LED is OFF during line following
  digitalWrite(LED_PIN, HIGH);

  unsigned long currentTime = millis();

  // Read sensors at regular intervals
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;

    // Get line position with multiple readings
    float rawPosition = readLinePosition();

    if (rawPosition >= 0) {
      // Line detected - use direct position
      currentPosition = rawPosition;
      lineDetected = true;
      lineDetectedTime = currentTime;
      lastLineDetectionTime = currentTime;  // Track when we last detected line (for dotted line detection)
    } else {
      // No line detected
      lineDetected = false;
    }
  }

  // Execute PID control at regular intervals
  if (currentTime - lastPidTime >= PID_INTERVAL) {
    lastPidTime = currentTime;

    // Check if line was detected recently (within last 200ms - increased for dashed lines)
    if (lineDetected && (currentTime - lineDetectedTime) < 200) {
      // Line is detected - normal PID control
      inRecoveryMode = false;

      // Reset dotted line mode if we were in it
      // This ensures clean state when we successfully find the line again
      if (inDottedLineMode) {
        inDottedLineMode = false;    // Exit dotted line mode
        dottedLineForwardCount = 0;  // Reset the step counter for next time

        // Track successful dotted line completion
        lastDottedLinePattern = true;
        dottedLinePatternCount++;

        // Reset pattern counter if too much time has passed
        if ((currentTime - lastLineDetectionTime) > PATTERN_RESET_TIME) {
          dottedLinePatternCount = 0;
        }
      }

      // ===== THROTTLE MANAGEMENT =====
      // Track when robot gets on line
      if (!wasOnLine) {
        onLineStartTime = currentTime;
        wasOnLine = true;
      }

      // Gradually increase throttle if robot has been on line long enough
      if (wasOnLine && (currentTime - onLineStartTime) >= THROTTLE_INTERVAL && (currentTime - lastThrottleTime) >= THROTTLE_INTERVAL) {
        if (currentThrottle < MAX_THROTTLE) {
          currentThrottle += THROTTLE_INCREMENT;
          lastThrottleTime = currentTime;
          updateAdaptivePID();  // Update PID values for new throttle
        }
      }

      // Calculate error from center position for curved array (3000 = exactly at sensor M0, PA0)
      // Curved array provides better precision for turns, so we can use more aggressive error calculation
      error = CURVED_ARRAY_CENTER - currentPosition;

      // Apply adaptive deadband to reduce jitter from small errors
      int currentDeadband = getAdaptiveDeadband();
      if (abs(error) < currentDeadband) {
        error = 0;     // Ignore very small errors
        integral = 0;  // Reset integral when centered
      }

      // Update turn direction memory based on current position for curved array
      // Only update direction when line is significantly off-center (not just oscillation)
      if (currentPosition < 1000) lastTurnDirection = -1;      // Left turn (line on left side)
      else if (currentPosition > 5000) lastTurnDirection = 1;  // Right turn (line on right side)
      else if (abs(currentPosition - CURVED_ARRAY_CENTER) < 500) {
        lastTurnDirection = 0;  // Reset to straight when near center
      }

      // PID calculations with time-based integral
      float deltaTime = PID_INTERVAL / 1000.0;  // Convert to seconds
      integral += error * deltaTime;

      // Limit integral windup
      integral = constrain(integral, -500, 500);

      float derivative = (error - lastError) / deltaTime;
      float rawCorrection = Kp * error + Ki * integral + Kd * derivative;

      // Limit correction to prevent excessive steering that slows the robot
      correction = constrain(rawCorrection, -MAX_CORRECTION, MAX_CORRECTION);

      lastError = error;

      // Normal PID control with forward motion using dynamic throttle
      leftSpeed = currentThrottle - correction;
      rightSpeed = currentThrottle + correction;

      // Allow full range of motor speeds for responsive control
      // Full reverse capability for sharp turns
      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      // Apply motor speeds with constraints
      leftMotor(leftSpeed);
      rightMotor(rightSpeed);



    } else {
      // No line detected - check for dotted line pattern first
      // This is the key improvement: instead of immediately turning when line is lost,
      // we first check if this might be a dotted line (brief white space)
      handleDottedLineMode();

      // If we're in dotted line mode, skip normal recovery
      // The dotted line handler is managing the robot's movement
      if (inDottedLineMode) {
        return;  // Continue with dotted line mode - don't enter normal recovery
      }

      // No line detected and not in dotted line mode - enter normal recovery mode
      // This handles cases where the line is truly lost (not just a dotted line gap)
      if (!inRecoveryMode) {
        inRecoveryMode = true;
        recoveryStartTime = currentTime;
      }

      // ===== THROTTLE RESET =====
      // Reset throttle when line is lost (after the 100ms grace period)
      if (wasOnLine) {
        currentThrottle = MIN_THROTTLE;  // Reset to base speed
        wasOnLine = false;               // Mark as off-line
        updateAdaptivePID();             // Update PID values for reset throttle
      }

      // Check if we should attempt recovery based on edge sensor memory
      unsigned long timeSinceLoss = currentTime - lineDetectedTime;

      if (timeSinceLoss < 3000) {  // Extended window for dashed line handling

        // Enhanced intersection handling
        if (intersectionDetected && (currentTime - lastIntersectionTime) < INTERSECTION_MEMORY_TIME) {
          // Near intersection - use gentle search pattern
          if (timeSinceLoss < 200) {
            // Small corrective movement
            if (lastTurnDirection == -1) {
              leftMotor(-30);
              rightMotor(80);
            } else if (lastTurnDirection == 1) {
              leftMotor(80);
              rightMotor(-30);
            } else {
              leftMotor(50);
              rightMotor(50);  // Forward
            }
          } else {
            // Continue with normal recovery
          }
        }
        // Enhanced edge sensor memory recovery for curved array
        // Curved array provides better edge detection, so we can extend the memory window
        bool recentLeftEdge = leftEdgeDetected && (currentTime - leftEdgeTime) < 500;     // Extended from 300ms
        bool recentRightEdge = rightEdgeDetected && (currentTime - rightEdgeTime) < 500;  // Extended from 300ms

        if (recentLeftEdge && !recentRightEdge) {
          // Left edge (L3) detected recently - gentle left turn
          if (timeSinceLoss < 400) {
            leftMotor(-40);
            rightMotor(80);  // Gentle left turn
          } else {
            leftMotor(-60);
            rightMotor(100);  // Slightly more aggressive
          }
          lastTurnDirection = -1;
        } else if (recentRightEdge && !recentLeftEdge) {
          // Right edge (R3) detected recently - gentle right turn
          if (timeSinceLoss < 400) {
            leftMotor(80);
            rightMotor(-40);  // Gentle right turn
          } else {
            leftMotor(100);
            rightMotor(-60);  // Slightly more aggressive
          }
          lastTurnDirection = 1;
        } else {
          // No edge sensors detected - GO STRAIGHT FORWARD
          // This prevents unnecessary turning on dashed lines
          if (timeSinceLoss < 800) {
            // Continue straight for first 800ms
            leftMotor(MIN_THROTTLE + 20);
            rightMotor(MIN_THROTTLE + 20);
          } else if (lastTurnDirection != 0) {
            // Only after 800ms, use last known direction for gentle search
            if (lastTurnDirection == -1) {
              leftMotor(40);
              rightMotor(80);  // Gentle left search
            } else {
              leftMotor(80);
              rightMotor(40);  // Gentle right search
            }
          } else {
            // No direction memory - continue straight
            leftMotor(MIN_THROTTLE + 10);
            rightMotor(MIN_THROTTLE + 10);
          }
        }
      } else {
        // Recovery timeout - stop and reset
        stopMotors();
        integral = 0;
        currentPosition = CURVED_ARRAY_CENTER;  // Reset to center position for curved array
        leftEdgeDetected = false;
        rightEdgeDetected = false;

        // Reset intersection detection after timeout
        if ((currentTime - lastIntersectionTime) > INTERSECTION_MEMORY_TIME) {
          intersectionDetected = false;
        }
      }
    }
  }
}

// ===== CURVED ARRAY OPTIMIZATION FUNCTIONS =====
void calculateCurvedArrayParameters() {
  // Calculate optimal parameters based on curved array geometry
  // This function can be called during setup or when parameters need updating

  // Adjust error deadband based on curved array precision
  // Curved array provides better precision, so we can use smaller deadband
  ERROR_DEADBAND = 8;  // Reduced from 10 for better curved array response

  // Adjust max correction based on curved array's enhanced turn detection
  MAX_CORRECTION = 600;  // Increased for more responsive curved array control

  // Adjust throttle parameters for curved array performance
  THROTTLE_INCREMENT = 15;  // Increased for faster acceleration with curved array
  THROTTLE_INTERVAL = 150;  // Reduced for more responsive curved array control
}

// ===== ENHANCED DOTTED LINE DETECTION FUNCTION =====
bool detectDottedLinePattern() {
  // This function identifies the characteristic pattern of dotted lines:
  // Line detected → Brief white space → Line detected again

  unsigned long currentTime = millis();
  bool recentlyHadLine = (currentTime - lastLineDetectionTime) < 300;  // Increased window for curves

  // Pattern 1: Recent line loss (basic dotted line)
  if (recentlyHadLine && !lineDetected) {
    return true;
  }

  // Pattern 2: Intersection detection (multiple sensors active)
  if (intersectionDetected && (currentTime - lastIntersectionTime) < INTERSECTION_MEMORY_TIME) {
    return true;
  }

  // Pattern 3: Curved dotted line pattern (extended gaps)
  if (lastDottedLinePattern && (currentTime - lastLineDetectionTime) < 500) {
    dottedLinePatternCount++;
    if (dottedLinePatternCount >= DOTTED_LINE_PATTERN_THRESHOLD) {
      return true;
    }
  }

  // Pattern 4: Edge sensor memory (line near edges)
  bool recentLeftEdge = leftEdgeDetected && (currentTime - leftEdgeTime) < 400;
  bool recentRightEdge = rightEdgeDetected && (currentTime - rightEdgeTime) < 400;

  if ((recentLeftEdge || recentRightEdge) && !lineDetected) {
    return true;
  }

  return false;  // Not a dotted line pattern
}

// ===== FORWARD-LOOKING LINE DETECTION =====
void handleDottedLineMode() {
  // This function manages the robot's behavior when traversing dotted lines
  // Instead of immediately turning when line is lost, it continues forward
  // to check if there's a line ahead (which is typical of dotted lines)

  unsigned long currentTime = millis();

  // STEP 1: Check if we should enter dotted line mode
  // Only enter if we're not already in it and we detect a dotted line pattern
  if (!inDottedLineMode && detectDottedLinePattern()) {
    inDottedLineMode = true;            // Set the mode flag
    dottedLineStartTime = currentTime;  // Record when we entered this mode
    dottedLineForwardCount = 0;         // Reset the forward step counter
    Serial1.println("Entering dotted line mode - continuing forward");
  }

  // STEP 2: Handle behavior while in dotted line mode
  if (inDottedLineMode) {
    // EXIT CONDITION 1: Check if we found line again
    // This is the success case - we found the line after the gap
    if (lineDetected) {
      inDottedLineMode = false;  // Exit dotted line mode
      Serial1.println("Line found - exiting dotted line mode");
      return;  // Skip normal recovery mode
    }

    // EXIT CONDITION 2: Check if we've been in dotted line mode too long
    // This prevents the robot from going too far if there's no line ahead
    if ((currentTime - dottedLineStartTime) > DOTTED_LINE_TIMEOUT) {
      inDottedLineMode = false;  // Exit dotted line mode
      Serial1.println("Dotted line timeout - entering recovery mode");
      return;  // Fall back to normal recovery
    }

    // EXIT CONDITION 3: Check if we've moved forward enough steps
    // This prevents the robot from going too far off course
    if (dottedLineForwardCount >= MAX_DOTTED_FORWARD_STEPS) {
      inDottedLineMode = false;  // Exit dotted line mode
      Serial1.println("Max forward steps reached - entering recovery mode");
      return;  // Fall back to normal recovery
    }

    // STEP 3: Continue forward movement in dotted line mode
    dottedLineForwardCount++;  // Increment the step counter

    // Enhanced direction prediction for dotted line curves
    float lastKnownError = CURVED_ARRAY_CENTER - currentPosition;

    // Adaptive correction based on dotted line pattern
    float correctionFactor = 0.6;  // Start with moderate correction

    // Increase correction for curved dotted lines
    if (dottedLinePatternCount >= DOTTED_LINE_PATTERN_THRESHOLD) {
      correctionFactor = 0.8;  // More aggressive for confirmed dotted line curves
    }

    // Special handling for intersections
    if (intersectionDetected && (currentTime - lastIntersectionTime) < 500) {
      correctionFactor = 0.4;  // Gentle correction near intersections
    }

    correction = constrain(lastKnownError * correctionFactor, -250, 250);

    // Adaptive forward speed based on pattern complexity
    int forwardSpeed = MIN_THROTTLE + 15;  // Slightly reduced for better control

    // Increase speed for straight dotted lines, reduce for curves
    if (abs(lastKnownError) < 500) {  // Straight ahead
      forwardSpeed += 10;
    } else {  // Curved path
      forwardSpeed -= 5;
    }

    leftSpeed = forwardSpeed - correction;
    rightSpeed = forwardSpeed + correction;

    // Apply motor speeds with full range capability
    leftMotor(constrain(leftSpeed, -255, 255));
    rightMotor(constrain(rightSpeed, -255, 255));

    return;  // Skip normal recovery mode - we're handling this ourselves
  }
}

void setup() {
  analogReadResolution(12);

  // Initialize sensor pins for curved 7-sensor array
  for (int i = 0; i < 7; i++) pinMode(sensorPins[i], INPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Turn LED on to indicate ready

  // Initialize Serial1 for calibration output
  Serial1.begin(115200);

  // Wait for system to be ready
  delay(1000);

  // Perform sensor calibration on boot
  performSensorCalibration();

  // Display calibration results
  displayCalibrationStatus();

  // Calculate curved array specific parameters
  calculateCurvedArrayParameters();

  // Initialize adaptive PID values
  updateAdaptivePID();

  Serial1.println("=== ROBOT READY FOR LINE FOLLOWING ===");
  Serial1.println("Place robot on black line to start...");
}

void loop() {
  // Execute main line following logic
  executeLineFollowing();
  // Print debugging information to Serial1
  Serial1.print("Position: ");
  Serial1.print(currentPosition);
  Serial1.print("  Correction: ");
  Serial1.print(correction);
  Serial1.print("  LeftSpeed: ");
  Serial1.print(leftSpeed);
  Serial1.print("  RightSpeed: ");
  Serial1.println(rightSpeed);
}
