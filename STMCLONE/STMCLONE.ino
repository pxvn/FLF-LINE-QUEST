/*
  =====================================================================
  STMCLONE — faithful port of the STM32 reference's search-pattern logic
  =====================================================================
  This is a direct port of the referenced STM32 code's control flow and
  state machine (ON_LINE / LOST_LINE / FULL_STOP, and the fixed search
  pattern INITIAL_STOP -> TILT_RIGHT -> TILT_LEFT -> MOVE_FORWARD ->
  TURN_AROUND -> FINAL_FORWARD_MOVE) onto our hardware. The whole point of
  this file is TURN_AROUND: a real, dedicated 180-degree reverse pivot
  (one wheel forward, one wheel reverse, same as the reference), which is
  what our other line-follower files deliberately avoided doing as a
  general search technique. Here it's kept, verbatim in spirit, because
  that's specifically what was asked for.

  What's identical to the reference:
    - LineState / SearchPhase enums and the exact same phase sequence,
      constants, and timings (SEARCH_SPEED, TURN_SPEED, WAIT_TIME,
      FORWARD_TIME, TILT_TIME, TURN_TIME, PAUSE_DURATION, FINAL_FORWARD,
      LOST_LINE_TIMEOUT, BASE_SPEED, PID gains).
    - The rotate-in-place calibration (spin for 6s while tracking each
      sensor's min/max, threshold = midpoint).
    - The ON_LINE / LOST_LINE branching logic and weighted-average line
      position calculation.

  What's changed for our hardware:
    - 14 sensors read through the CD74HC4067 mux (the reference reads 7-8
      sensors directly on separate analog pins) instead of direct pins.
      Weights scale 0..13000 in steps of 1000; center is (NUM_SENSORS-1)*
      500 = 6500. The reference's 7-sensor version used a hardcoded
      "position - 2000" offset that isn't actually centered for its own
      0-6000 weight range (looks like a leftover from an earlier tuning
      pass, not deliberate) -- that's the one thing NOT carried over
      as-is, since copying a miscalibrated center would make this bot
      hug one side of the line by design. Everything else is untouched.
    - Motors driven through our BTS7960 (setLeftMotor/setRightMotor style,
      via ledcWrite) instead of the reference's TB6612 direction-pin pair,
      but the calling code (leftMotor()/rightMotor()/setMotorSpeeds()) is
      named and used exactly the same way so the state machine above it
      didn't need to change at all.
    - Added: a single start/stop button (calibrates then runs on first
      press, stops on the next), and buzzer + LED status feedback -- the
      reference has neither, it just auto-runs from power-on.

  Board: Waveshare ESP32-S3-Pico. No OLED, no menu -- this is meant to
  stay a minimal, direct clone, not merged into the fuller FLF_V3/V4/V3.3
  firmware.
  =====================================================================
*/

#include <Adafruit_NeoPixel.h>

// ---- Multiplexer (CD74HC4067) ----
#define MUX_SIG   1
#define MUX_S3    2
#define MUX_S2    4
#define MUX_S1    5
#define MUX_S0    6
#define NUM_SENSORS 14
const uint8_t muxChannelForSensor[NUM_SENSORS] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};

// ---- Motors (BTS7960) ----
#define R_RPWM 15
#define R_LPWM 16
#define L_LPWM 33
#define L_RPWM 34
#define PWM_FREQ 20000
#define PWM_RES  8

// ---- LED (WS2812) ----
#define LED_PIN 7
#define NUM_LEDS 2
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---- Buzzer ----
#define BUZZER_PIN 37

// ---- Start/Stop button (single button, our SW3/OK pin) ----
#define START_BTN 12

// ===== Motor speed constants (same as reference) =====
const int BASE_SPEED = 130;
const int MAX_SPEED  = 255;
const int MIN_SPEED  = -255;

// ===== PID Constants (same as reference) =====
float Kp = 0.05;
float Ki = 0.2;
float Kd = 1;
float lastError = 0;
float integral = 0;

// ===== Line following states (same as reference) =====
enum LineState { ON_LINE, LOST_LINE, FULL_STOP };
const unsigned long LOST_LINE_TIMEOUT = 200;
unsigned long lastLineTime = 0;
LineState currentState = LOST_LINE;

float lastCorrection = 0;
bool allWhite = false;

// ===== Search pattern constants (same as reference) =====
const int SEARCH_SPEED = 130;
const int TURN_SPEED   = 140;
const unsigned long WAIT_TIME      = 500;
const unsigned long FORWARD_TIME   = 350;
const unsigned long TILT_TIME      = 370;
const unsigned long TURN_TIME      = 330;  // TURN_AROUND duration -- the 180
const unsigned long PAUSE_DURATION = 500;
const unsigned long FINAL_FORWARD  = 700;

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
unsigned long phaseStartTime = 0;
bool pauseInProgress = false;

// ===== Sensor data =====
int currentSensorValues[NUM_SENSORS];
int sensorMin[NUM_SENSORS], sensorMax[NUM_SENSORS], sensorThreshold[NUM_SENSORS];

// ===== Run state (our addition: start/stop button gates everything) =====
bool running = false;

// ---- single-button debounce ----
bool btnLast = HIGH, btnStable = HIGH;
unsigned long btnChangeAt = 0;
bool buttonPressed() {
  bool reading = digitalRead(START_BTN);
  bool edge = false;
  if (reading != btnLast) btnChangeAt = millis();
  if (millis() - btnChangeAt > 30 && reading != btnStable) {
    btnStable = reading;
    if (btnStable == LOW) edge = true;
  }
  btnLast = reading;
  return edge;
}

// ---- buzzer + LED (our addition) ----
void beep(int freq, int dur) {
  ledcWriteTone(BUZZER_PIN, freq);
  delay(dur);
  ledcWriteTone(BUZZER_PIN, 0);
}
void setLED(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}

// ---- mux sensor read ----
void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
  digitalWrite(MUX_S3, (ch >> 3) & 1);
  delayMicroseconds(8);
}
int readSensorRaw(int i) {
  selectMuxChannel(muxChannelForSensor[i]);
  return analogRead(MUX_SIG);
}

// ===== Motor functions -- same names/calling convention as the
// reference's leftMotor()/rightMotor()/setMotorSpeeds(), reimplemented
// underneath for BTS7960 instead of TB6612 =====
void leftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(L_RPWM, speed); ledcWrite(L_LPWM, 0); }
  else            { ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, -speed); }
}
void rightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(R_RPWM, speed); ledcWrite(R_LPWM, 0); }
  else            { ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, -speed); }
}
void stopMotors() { leftMotor(0); rightMotor(0); }
void setMotorSpeeds(int l, int r) { leftMotor(l); rightMotor(r); }

// ===== Calibration -- same logic as the reference: rotate in place for
// 6s while tracking each sensor's min/max, threshold = midpoint =====
void calibrateSensors() {
  Serial.println("Calibrating (rotating)...");
  setLED(255, 200, 0); // amber during calibration

  for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }

  unsigned long startTime = millis();
  bool blinkOn = false;
  unsigned long lastBlink = 0;
  while (millis() - startTime < 6000) {
    setMotorSpeeds(-180, 180); // spin clockwise, same as reference
    for (int i = 0; i < NUM_SENSORS; i++) {
      int val = readSensorRaw(i);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    if (millis() - lastBlink > 150) { // fast blink to show calibration in progress
      lastBlink = millis();
      blinkOn = !blinkOn;
      setLED(blinkOn ? 255 : 0, blinkOn ? 200 : 0, 0);
    }
  }
  stopMotors();

  Serial.println("Calibration done:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    Serial.print("S"); Serial.print(i);
    Serial.print(": min="); Serial.print(sensorMin[i]);
    Serial.print(" max="); Serial.print(sensorMax[i]);
    Serial.print(" thr="); Serial.println(sensorThreshold[i]);
  }
  beep(1800, 150);
}

// ===== Line position -- same weighted-average logic as the reference,
// scaled to 14 sensors (0..13000, center 6500) =====
float getLinePosition() {
  float sum = 0, weightedSum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int rawVal = readSensorRaw(i);
    currentSensorValues[i] = (rawVal < sensorThreshold[i]) ? 1 : 0; // 1 = black line
    if (currentSensorValues[i] == 1) {
      weightedSum += 1000.0f * (i * 1000.0f);
      sum += 1000.0f;
    }
  }
  float position = (sum > 0) ? (weightedSum / sum) : ((NUM_SENSORS - 1) * 500.0f); // default center
  return position;
}

// ===== Line state -- same logic as the reference =====
LineState getLineState() {
  int sensorsOnLine = 0;
  allWhite = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (currentSensorValues[i] == 1) {
      sensorsOnLine++;
      allWhite = false;
      lastLineTime = millis();
    }
  }
  if (sensorsOnLine == 0) {
    if (millis() - lastLineTime > LOST_LINE_TIMEOUT) return LOST_LINE;
    return ON_LINE; // temporarily assume still on line, same as reference
  }
  return ON_LINE;
}

String getStateName(LineState state) {
  switch (state) {
    case ON_LINE: return "ON_LINE";
    case LOST_LINE: return "LOST_LINE";
    case FULL_STOP: return "FULL_STOP";
    default: return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  analogReadResolution(12);

  pinMode(START_BTN, INPUT_PULLUP);

  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  stopMotors();

  pinMode(BUZZER_PIN, OUTPUT);
  ledcAttach(BUZZER_PIN, 2000, 8);

  strip.begin();
  strip.setBrightness(80);
  strip.show();
  setLED(0, 0, 40); // idle blue: waiting for start press

  beep(1800, 100);
  Serial.println("STMCLONE ready. Press the button to calibrate and start.");
}

void loop() {
  if (buttonPressed()) {
    if (!running) {
      calibrateSensors();
      running = true;
      currentState = LOST_LINE;
      lastLineTime = millis();
      searchPatternStarted = false;
      integral = 0;
      lastError = 0;
      lastCorrection = 0;
      setLED(0, 255, 0);
      beep(1500, 80); delay(100); beep(2400, 120);
      Serial.println("STARTED");
    } else {
      running = false;
      stopMotors();
      setLED(0, 0, 40);
      beep(900, 150);
      Serial.println("STOPPED");
    }
  }

  if (!running) { stopMotors(); return; }

  LineState newState = getLineState();
  float position = getLinePosition();

  Serial.print("State: "); Serial.print(getStateName(newState));
  Serial.print(" Pos: "); Serial.println(position);

  int leftSpeed = 0, rightSpeed = 0;

  switch (newState) {
    case FULL_STOP:
      stopMotors();
      running = false;
      setLED(255, 0, 0);
      beep(400, 300);
      break;

    case LOST_LINE:
      setLED(255, 40, 0); // amber-red while searching
      if (millis() - lastLineTime <= LOST_LINE_TIMEOUT) {
        // Same as reference: ride out a brief loss with the last correction
        leftSpeed  = BASE_SPEED + lastCorrection;
        rightSpeed = BASE_SPEED - lastCorrection;
        leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
        rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
        setMotorSpeeds(leftSpeed, rightSpeed);
      } else {
        // ---- Fixed search pattern, same phases/timings as the reference.
        // TURN_AROUND here is the actual 180-degree reverse pivot. ----
        if (!searchPatternStarted) {
          searchStartTime = millis();
          phaseStartTime = millis();
          searchPatternStarted = true;
          searchPhase = INITIAL_STOP;
          pauseInProgress = false;
        }

        unsigned long phaseElapsedTime = millis() - phaseStartTime;

        switch (searchPhase) {
          case INITIAL_STOP:
            stopMotors();
            if (phaseElapsedTime >= WAIT_TIME) {
              phaseStartTime = millis();
              searchPhase = TILT_RIGHT;
            }
            break;

          case TILT_LEFT:
            if (!pauseInProgress) {
              setMotorSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
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

          case TURN_AROUND: // the 180
            if (!pauseInProgress) {
              setMotorSpeeds(-TURN_SPEED, TURN_SPEED);
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
            if (phaseElapsedTime >= FINAL_FORWARD) {
              phaseStartTime = millis();
              searchPhase = INITIAL_STOP;
            }
            break;
        }
      }
      break;

    case ON_LINE: {
      setLED(0, 255, 0);
      searchPatternStarted = false;
      float error;
      if (allWhite) {
        error = lastError;
      } else {
        error = position - ((NUM_SENSORS - 1) * 500.0f); // true center for our 0..13000 range
        lastError = error;
      }

      integral += error;
      integral = constrain(integral, -10000, 10000);
      float derivative = error - lastError;

      float correction = (Kp * error + Ki * integral + Kd * derivative) / 100.0f;
      lastCorrection = correction;

      leftSpeed  = BASE_SPEED + correction;
      rightSpeed = BASE_SPEED - correction;
      leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
      rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

      setMotorSpeeds(leftSpeed, rightSpeed);
      break;
    }
  }

  currentState = newState;
}
