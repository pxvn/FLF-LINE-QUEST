              /*
  =====================================================================
  LINE FOLLOWER ROBOT — Waveshare ESP32-S3-Pico  (V3.3: maze arena pass)
  =====================================================================
  Forked from FLF_V3 (kept as the stable baseline). Adds two things for
  running on an actual junction/maze arena, both scoped to lineFollowLoop():
    - Junction/lost-line handling: the outer 3 sensors each side are sticky
      "which way did I last see the line" memory (LEFTMOST_N/RIGHTMOST_N).
      When the line disappears entirely, the bot coasts straight for
      LINE_LOST_COAST_MS (handles a junction crossbar or a gap without any
      turn decision at all, most of the time) before committing to a turn
      using that memory -- not a hardcoded direction, and not last-drift-
      sign guessing like plain V3 did.
    - Edge sensors also get an EDGE_GAIN multiplier in the weighted-average
      PID error itself, so the bot reacts more decisively as the line
      approaches the outer sensors (entering a turn) instead of just at
      the moment it's fully lost.
    - End Zone detection: a sustained (END_ZONE_HOLD_MS), wide
      (END_ZONE_MIN_SENSORS+ of 14) black reading -- distinct from a brief
      junction crossing -- stops the bot, lights the LED white, and plays a
      finish tune. LED stays in normal mode colors the rest of the run, per
      the "don't glow until the end zone" rule.

  Original V3 feature list:
    - OLED + 4-button menu UI (button roles are logical, see "BUTTON
      HANDLING" below — physical wiring stays fixed, roles are mapped
      onto it so relabeling the enclosure never means touching every
      screen's code)
    - Sensor calibration (14x TCRT5000 via CD74HC4067 mux), with a
      line-color (dark-on-light / light-on-dark) invert toggle so the
      PID math can't silently steer backwards on the wrong wiring
    - Motor test mode: cycle Left/Right/Both, hold to drive fwd/rev,
      live encoder counts + RPM (or counts/sec if RPM isn't configured)
      for 2x N30 motors w/ encoder, BTS7960 drivers (~1000 RPM motors)
    - PID line following, control loop decoupled from the OLED refresh
      so a slow I2C frame push can't throttle steering reaction time
    - WS2812 RGB LED (x2): breathing idle color, mode colors, red-flash
      warning when the line is lost, plus a manually-adjustable idle
      color from the menu
    - Passive buzzer feedback beeps, incl. distinct fwd/rev tones in
      motor test
    - Calibration + LED color + PID + line polarity saved to flash
      (Preferences/NVS)
    - Optional Serial telemetry while line following (DEBUG_SERIAL)

  Required libraries (Install via Library Manager):
    - Adafruit GFX Library
    - Adafruit SSD1306
    - Adafruit NeoPixel
    (Preferences.h is built into the ESP32 Arduino core)

  Board: Waveshare ESP32-S3-Pico ("ESP32S3 Dev Module")
  Arduino-ESP32 core: 3.x (uses ledcAttach/ledcWrite-by-pin API and tone())

  TODO before trusting Motor Test's RPM readout: set
  ENCODER_COUNTS_PER_REV below to your encoder's actual counts per
  output-shaft revolution (quadrature counts x gearbox ratio). Until
  then the screen shows raw counts/sec, which is always accurate.
  =====================================================================
*/

#include <Wire.h>
#include <math.h>
#include <string.h>  // memcmp, used to verify a save actually landed in flash
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// =====================================================================
//  TYPE DEFINITIONS
//  (Kept at the very top: the Arduino IDE auto-generates function
//   prototypes and inserts them right after the includes, so any
//   struct/enum used as a function parameter or return type must
//   already be defined by this point or you'll get "does not name
//   a type" errors.)
// =====================================================================
enum State {
  STATE_MENU,
  STATE_CALIBRATION,
  STATE_MOTOR_TEST,
  STATE_LINE_FOLLOW,
  STATE_LED_COLOR,
  STATE_PID_TUNE,
  STATE_SENSOR_TEST
};

enum MotorTestTarget { MT_LEFT, MT_RIGHT, MT_BOTH };
enum CalibMode { CALIB_MANUAL, CALIB_AUTO };
enum CalibPhase { CALIB_SELECT_MODE, CALIB_RUNNING, CALIB_AUTO_DONE };

struct RGB { uint8_t r, g, b; };

struct Button {
  uint8_t pin;
  bool lastReading = HIGH;
  bool stableState = HIGH;
  unsigned long lastChange = 0;
};

// =====================================================================
//  PIN DEFINITIONS  (from your wiring table)
// =====================================================================
// ---- Multiplexer (CD74HC4067) ----
#define MUX_SIG   1
#define MUX_S3    2
#define MUX_S2    4
#define MUX_S1    5
#define MUX_S0    6

// ---- OLED (I2C) ----
#define OLED_SDA  8
#define OLED_SCL  9
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64

// ---- Buttons (physical pins; logical roles are assigned in the
//      BUTTON HANDLING section below, not here) ----
#define SW1       10
#define SW2       11
#define SW3       12
#define SW4       13

// ---- Right Motor Driver (BTS7960) ----
#define R_RPWM    15   // "PWMR"
#define R_LPWM    16   // "PWML"

// ---- Left Motor Driver (BTS7960) ----
#define L_LPWM    33   // "PWML"
#define L_RPWM    34   // "PWMR"

// ---- Encoders ----
#define L_ENC_A   17
#define L_ENC_B   18
#define R_ENC_A   35
#define R_ENC_B   36

// ---- RGB LED (WS2812) ----
#define LED_PIN   7
#define NUM_LEDS  2

// ---- Buzzer ----
#define BUZZER_PIN 37

// =====================================================================
//  CONSTANTS
// =====================================================================
#define NUM_SENSORS 14
const uint8_t muxChannelForSensor[NUM_SENSORS] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};

#define PWM_FREQ  20000
#define PWM_RES   8      // 0-255

// Set to your encoder's counts per output-shaft revolution (quadrature
// counts x gear ratio) to get real RPM in Motor Test. 0 = unknown -> the
// screen falls back to raw counts/sec, which needs no calibration.
const int ENCODER_COUNTS_PER_REV = 0;

// Print err/L/R/lineFound over Serial while line following (115200 baud),
// throttled to the same rate as the OLED. Handy for PID tuning.
#define DEBUG_SERIAL 1

// PID defaults (tune from the "PID Tune" submenu, or edit here)
float Kp = 0.06;
float Ki = 0.0006;
float Kd = 0.4;
int baseSpeed = 150;     // 0-255, forward cruise speed while line following
int testSpeed = 180;     // 0-255, manual jog speed used by Motor Test; tune
                          // it from the Settings screen (adjustable there
                          // as its own parameter, see "TestSpd")

int uTurnMinMs = 600;    // blind-pivot duration for the checkpoint U-turn: sensors are
                          // ignored entirely until this elapses, so the checkpoint's own
                          // crossing mark can't abort the turn a few degrees in. Tune from
                          // Settings ("UturnMs") until the pivot lands a true ~180.

const int ON_THRESH = 500; // per-sensor lineWeight() above this = "on line";
                            // used by calibration's live tick row and by
                            // junction edge-sensing below

// Percentage trimmed off each end of the calibrated span in
// normalizeSensors() -- see the note there.
const int NORM_MARGIN_PCT = 8;

// Error conditioning -- see lineFollowLoop. Declared up here (not beside
// the PID state) because saveSettings()/loadSettings() reference
// errDeadband, and Arduino only auto-prototypes functions, not variables.
float errorFiltered = 0;
int errDeadband = 150;      // +/- counts around centre treated as centre
                             // (full deflection is 6500). Settings: "Deadband".
const int ERR_EMA_DEN = 3;  // low-pass on the error itself; 1 = off

// ---- Junction / lost-line handling (see lineFollowLoop) ----
const int EDGE_COUNT = 3;             // outer N sensors each side used for turn memory + gain boost
const float EDGE_GAIN = 1.6f;         // extra weight multiplier on those outer sensors in the PID error
const unsigned long LINE_LOST_COAST_MS = 200; // coast straight this long before committing to a turn

// ---- End Zone detection (see lineFollowLoop) ----
const int END_ZONE_MIN_SENSORS = 12;        // out of 14 must read "on" simultaneously
const unsigned long END_ZONE_HOLD_MS = 400; // ...sustained this long (a normal junction crossing is brief)

// =====================================================================
//  GLOBAL OBJECTS
// =====================================================================
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

// =====================================================================
//  STATE MACHINE
// =====================================================================
State currentState = STATE_MENU;

// Cross-mode flags (declared here so menu code below can reference them)
CalibMode calibMode = CALIB_MANUAL;
CalibPhase calibPhase = CALIB_SELECT_MODE;
unsigned long calibAutoStart = 0;
const unsigned long CALIB_AUTO_DURATION_MS = 6000; // how long the auto-spin runs
const int CALIB_AUTO_SPEED = 140;                  // rotation speed while auto-calibrating
bool lineFollowStarting = false;

// Menu
const char* menuItems[] = {
  "Start Line Follow",
  "Calibrate Sensors",
  "Test Motors",
  "LED Color",
  "Settings",
  "Sensor Test"
};
const int menuCount = 6;
int menuIndex = 0;

// =====================================================================
//  SENSOR DATA
// =====================================================================
int sensorRaw[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorNorm[NUM_SENSORS]; // 0-1000 normalized, hi (calibrated max) -> 1000

// True: sensors read HIGHER raw values over the line than over the
// background (so a higher sensorNorm means "closer to the line").
// False: inverted wiring/reflectance -> the line reads LOWER. Toggle
// with UP while calibrating and watch "Line: OK/LOST" over the actual
// line to pick the right setting; it's persisted to flash.
bool lineIsDark = true;

// Which side wins when both outer sensor groups see line at once during a
// junction (see lastEdgeDir in lineFollowLoop). Set from the Settings menu
// before starting a run, not hardcoded; persisted to flash.
bool turnPriorityLeft = true;

// =====================================================================
//  ENCODER COUNTS
// =====================================================================
volatile long leftEncCount = 0;
volatile long rightEncCount = 0;

void IRAM_ATTR leftEncoderISR() {
  bool b = digitalRead(L_ENC_B);
  if (b) leftEncCount++; else leftEncCount--;
}
void IRAM_ATTR rightEncoderISR() {
  bool b = digitalRead(R_ENC_B);
  if (b) rightEncCount++; else rightEncCount--;
}

// Encoder rate tracking (updated ~10x/sec from loop(), read by Motor Test)
long lastLeftEncForRate = 0, lastRightEncForRate = 0;
unsigned long lastRateCalc = 0;
float leftCPS = 0, rightCPS = 0;   // counts per second (always valid)
float leftRPM = 0, rightRPM = 0;   // only meaningful if ENCODER_COUNTS_PER_REV is set

void updateEncoderRates() {
  unsigned long now = millis();
  unsigned long dt = now - lastRateCalc;
  if (dt < 100) return; // ~10 Hz
  long dl = leftEncCount - lastLeftEncForRate;
  long dr = rightEncCount - lastRightEncForRate;
  lastLeftEncForRate = leftEncCount;
  lastRightEncForRate = rightEncCount;
  lastRateCalc = now;

  float dtSec = dt / 1000.0f;
  leftCPS = dl / dtSec;
  rightCPS = dr / dtSec;
  if (ENCODER_COUNTS_PER_REV > 0) {
    leftRPM = (leftCPS / ENCODER_COUNTS_PER_REV) * 60.0f;
    rightRPM = (rightCPS / ENCODER_COUNTS_PER_REV) * 60.0f;
  }
}

// =====================================================================
//  LED COLORS
// =====================================================================
// Custom color the user picks for the idle/menu state (persisted)
uint8_t idleR = 0, idleG = 0, idleB = 60;

RGB colorForState(State s) {
  switch (s) {
    case STATE_CALIBRATION: return {255, 200, 0};   // amber/yellow
    case STATE_MOTOR_TEST:  return {160, 0, 255};   // purple
    case STATE_LINE_FOLLOW: return {0, 255, 0};     // green
    case STATE_LED_COLOR:   return {idleR, idleG, idleB};
    case STATE_PID_TUNE:    return {0, 200, 255};   // cyan
    case STATE_SENSOR_TEST: return {255, 0, 255};   // magenta
    default:                return {idleR, idleG, idleB}; // menu/idle
  }
}
void setLED(RGB c) {
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(c.r, c.g, c.b));
  strip.show();
}
void applyModeLED() { setLED(colorForState(currentState)); }

// Slow "breathing" idle color so the menu doesn't look static. Call every
// loop while in STATE_MENU; throttled internally.
void updateIdleBreath() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 30) return;
  lastUpdate = millis();
  float phase = (millis() % 3000) / 3000.0f * TWO_PI;
  float b = 0.25f + (sinf(phase) * 0.5f + 0.5f) * 0.75f; // 0.25..1.0
  setLED({(uint8_t)(idleR * b), (uint8_t)(idleG * b), (uint8_t)(idleB * b)});
}

// Solid green while the line is seen, flashing red while it's lost —
// call every loop from lineFollowLoop(); throttled internally.
void updateLineFollowLED(bool lineFound) {
  if (lineFound) { setLED({0, 255, 0}); return; }
  static bool flashOn = false;
  static unsigned long lastFlash = 0;
  if (millis() - lastFlash > 150) { flashOn = !flashOn; lastFlash = millis(); }
  setLED(flashOn ? RGB{255, 0, 0} : RGB{0, 0, 0});
}

// =====================================================================
//  BUZZER
// =====================================================================
// Deliberately NOT using tone(): on ESP32-S3 it silently shares the same
// 4 physical LEDC timers as motorsInit()'s PWM. If tone()'s auto-managed
// timer ever landed on the same timer as a motor channel, keying the
// buzzer would retune that timer's frequency out from under the motor
// PWM (audible whine + torque glitch on every beep) or the buzzer would
// simply fail to get a timer at all and stay silent — which matches
// "buzzer not working" while everything else on the board is fine.
// ledcWriteTone() reuses BUZZER_PIN's own explicitly-attached channel
// (given its own timer in setup(), separate from the 4 motor channels)
// and is non-blocking; buzzerService() below turns it off after `dur`.
unsigned long buzzerStopAt = 0;

void beep(int freq = 2000, int dur = 60) {
  ledcWriteTone(BUZZER_PIN, freq);
  buzzerStopAt = millis() + dur;
}
void buzzerService() {
  if (buzzerStopAt && (long)(millis() - buzzerStopAt) >= 0) {
    ledcWriteTone(BUZZER_PIN, 0);
    buzzerStopAt = 0;
  }
}
void beepOK()    { beep(2200, 50); }
void beepBack()  { beep(900, 70); }
void beepError() { beep(400, 250); }
void beepStart() { beep(1500, 80); delay(100); beep(2400, 120); }

// =====================================================================
//  BUTTON HANDLING (debounced, edge-detected)
// =====================================================================
Button btn1{SW1}, btn2{SW2}, btn3{SW3}, btn4{SW4};
const unsigned long DEBOUNCE_MS = 30;

// Returns true exactly once when button transitions HIGH->LOW (pressed)
bool wasPressed(Button &b) {
  bool reading = digitalRead(b.pin);
  bool pressedEdge = false;
  if (reading != b.lastReading) {
    b.lastChange = millis();
  }
  if ((millis() - b.lastChange) > DEBOUNCE_MS) {
    if (reading != b.stableState) {
      b.stableState = reading;
      if (b.stableState == LOW) pressedEdge = true;
    }
  }
  b.lastReading = reading;
  return pressedEdge;
}

// ---- Logical roles, mapped onto physical buttons here (and ONLY here).
// Physical wiring: SW1, SW2, SW3, SW4. Rotated one more step in the same
// direction as before (role@SW1 -> SW2, role@SW2 -> SW4, closing the loop
// role@SW4 -> SW1; SW3/OK untouched): SW1->DOWN, SW2->BACK, SW3->OK,
// SW4->UP. Every screen below calls up()/down()/select()/back() by role,
// never by SWx, so the physical layout can change again by editing only
// this block.
bool up()     { return wasPressed(btn4); }  // physical SW4
bool down()   { return wasPressed(btn1); }  // physical SW1
bool select() { return wasPressed(btn3); }  // physical SW3
bool back()   { return wasPressed(btn2); }  // physical SW2

// Raw (non-edge-triggered) hold state for the same UP/DOWN roles, used by
// Motor Test's hold-to-drive controls.
bool upHeld()   { return digitalRead(SW4) == LOW; }
bool downHeld() { return digitalRead(SW1) == LOW; }

// =====================================================================
//  MULTIPLEXER / SENSOR READING
// =====================================================================
// ---------------------------------------------------------------------
//  NOISE CONTROL -- why the raw values were fluctuating
// ---------------------------------------------------------------------
//  Four independent causes, all addressed below. They stack, which is why
//  the fluctuation looked severe and did not go away when the MCU was
//  swapped -- none of these are MCU faults:
//
//  1. SETTLING TIME. 8us was far too short. A TCRT5000 phototransistor
//     behind its pull-up is a HIGH-impedance source; the mux adds series
//     resistance and channel capacitance on top. The ESP32 ADC's
//     sample-and-hold capacitor must charge through all of that. Sample
//     too early and you read a value part-way between the PREVIOUS
//     channel and this one -- which looks like random noise, and is worst
//     on whichever sensor follows a very differently-lit one.
//  2. NO THROWAWAY READ. The first conversion after switching channels
//     still carries charge from the previous channel. One discarded read
//     flushes it.
//  3. NO FILTERING. A single sample has no defence against the 20kHz
//     motor PWM coupling into the analog rail. A MEDIAN is used rather
//     than an average on purpose: averaging drags impulse spikes into the
//     result, a median discards them outright.
//  4. THRESHOLD FLICKER. A sensor sitting near ON_THRESH flipped on/off
//     every cycle on a few counts of wobble. Hysteresis fixes the
//     DECISION even when the underlying value is genuinely marginal.
// ---------------------------------------------------------------------
const int MUX_SETTLE_US  = 120; // 8 -> 60 -> 120. The single most effective
                                 // knob: a high-impedance sensor behind a mux
                                 // needs real time to charge the ADC's S/H cap.
const int SENSOR_SAMPLES = 7;    // median of N, must be ODD -- see (3)

// Smoothing strength, adjustable from Settings as "Smooth" so it can be
// tuned on the bench without a reflash:
//   new = (old*(DEN-1) + fresh) / DEN
// 1  = off (raw median)      2 = light
// 4  = strong (default)      8 = very heavy, visibly laggy
// Higher = steadier values but slower reaction, so there is a real cost to
// overdoing it: too much lag and the bot oscillates for a different reason.
int sensorEmaDen = 4;

int sensorNoise[NUM_SENSORS];  // per-sensor peak-to-peak, for Sensor Test only

void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
  delayMicroseconds(MUX_SETTLE_US); // let mux + high-impedance sensor settle
}

void readAllSensors() {
  static bool seeded = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(muxChannelForSensor[i]);
    (void)analogRead(MUX_SIG); // (2) discard -- flushes the previous channel

    int buf[SENSOR_SAMPLES];
    int lo = 4095, hi = 0;
    for (int k = 0; k < SENSOR_SAMPLES; k++) {
      int v = analogRead(MUX_SIG);
      buf[k] = v;
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    sensorNoise[i] = hi - lo; // diagnostics only, never used for control

    for (int a = 1; a < SENSOR_SAMPLES; a++) { // insertion sort, N=5
      int v = buf[a], j = a - 1;
      while (j >= 0 && buf[j] > v) { buf[j + 1] = buf[j]; j--; }
      buf[j + 1] = v;
    }
    int fresh = buf[SENSOR_SAMPLES / 2]; // (3) median

    // (3b) light EMA on top of the median, seeded on the first pass so
    // startup does not ramp in from zero.
    if (!seeded || sensorEmaDen <= 1) sensorRaw[i] = fresh;
    else sensorRaw[i] = (sensorRaw[i] * (sensorEmaDen - 1) + fresh) / sensorEmaDen;
  }
  seeded = true;
}

void normalizeSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int lo = sensorMin[i], hi = sensorMax[i];
    if (hi <= lo) { sensorNorm[i] = 0; continue; }

    // Pull the calibrated ends inward by NORM_MARGIN% before mapping.
    // Reason: the recorded min/max are the most EXTREME samples seen during
    // calibration, so they are exactly the ones most likely to have been
    // noise spikes. Mapping against them stretches everyday readings across
    // a slightly-too-wide window, which amplifies the remaining wobble.
    // Trimming the ends makes the normalized value both steadier and more
    // likely to reach a clean 0 or 1000.
    long span = (long)(hi - lo);
    int  pad  = (int)(span * NORM_MARGIN_PCT / 100);
    int  loT  = lo + pad, hiT = hi - pad;
    if (hiT <= loT) { loT = lo; hiT = hi; } // degenerate span -- fall back

    int v = constrain(sensorRaw[i], loT, hiT);
    sensorNorm[i] = map(v, loT, hiT, 0, 1000);
  }
}

// "How much this sensor thinks it's on the line", 0-1000, correcting for
// wiring polarity via lineIsDark so the PID math always weights toward
// the line regardless of which way the calibrated min/max fell.
int lineWeight(int i) {
  return lineIsDark ? sensorNorm[i] : (1000 - sensorNorm[i]);
}

// (4) Hysteresis on the on/off decision: a sensor must climb past
// ON_THRESH+ON_HYST to switch ON, and fall below ON_THRESH-ON_HYST to
// switch OFF. In between it keeps its previous answer. This stops a
// sensor parked on the edge of the line from flickering every cycle and
// shaking the PID error, which is the part of the noise you actually FEEL
// in the driving.
const int ON_HYST = 80;
bool sensorOnState[NUM_SENSORS];

bool sensorIsOn(int i) {
  int w = lineWeight(i);
  if (sensorOnState[i]) { if (w < ON_THRESH - ON_HYST) sensorOnState[i] = false; }
  else                  { if (w > ON_THRESH + ON_HYST) sensorOnState[i] = true;  }
  return sensorOnState[i];
}

// =====================================================================
//  MOTOR CONTROL (BTS7960: separate forward/reverse PWM pins, no EN)
// =====================================================================
void motorsInit() {
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
}

void setRightMotor(int speed) { // -255..255
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(R_RPWM, speed); ledcWrite(R_LPWM, 0); }
  else            { ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, -speed); }
}
void setLeftMotor(int speed) { // -255..255
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(L_RPWM, speed); ledcWrite(L_LPWM, 0); }
  else            { ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, -speed); }
}
void stopMotors() { setLeftMotor(0); setRightMotor(0); }

// =====================================================================
//  PREFERENCES (persist calibration + LED color + PID + line polarity)
// =====================================================================
// Every save below reads back what it just wrote and compares — a "Save"
// press now actually PROVES the write landed instead of assuming it did.
// confirmSaveResult() gives an unmistakable pass/fail beep+flash so you
// don't need to trust silence: two rising chirps + a green flash means
// verified-good, one long low buzz + red flash means the write did NOT
// read back correctly (flash wear-out, NVS corruption, brownout, etc.)
void confirmSaveResult(bool ok) {
  RGB restore = colorForState(currentState);
  if (ok) {
    beep(2400, 40); delay(70); beep(2700, 40); delay(70);
    setLED({0, 255, 0}); delay(150);
  } else {
    beep(300, 350); delay(370);
    setLED({255, 0, 0}); delay(350);
  }
  setLED(restore);
}

bool saveCalibration() {
  prefs.begin("linefw", false);
  prefs.putBytes("smin", sensorMin, sizeof(sensorMin));
  prefs.putBytes("smax", sensorMax, sizeof(sensorMax));
  prefs.putBool("invert", lineIsDark);
  prefs.end();

  int chkMin[NUM_SENSORS], chkMax[NUM_SENSORS];
  prefs.begin("linefw", true);
  size_t r1 = prefs.getBytes("smin", chkMin, sizeof(chkMin));
  size_t r2 = prefs.getBytes("smax", chkMax, sizeof(chkMax));
  bool invertOk = prefs.getBool("invert", !lineIsDark) == lineIsDark;
  prefs.end();

  bool ok = r1 == sizeof(chkMin) && r2 == sizeof(chkMax) &&
            memcmp(chkMin, sensorMin, sizeof(chkMin)) == 0 &&
            memcmp(chkMax, sensorMax, sizeof(chkMax)) == 0 && invertOk;
  confirmSaveResult(ok);
  return ok;
}
void loadCalibration() {
  prefs.begin("linefw", true);
  size_t r1 = prefs.getBytes("smin", sensorMin, sizeof(sensorMin));
  size_t r2 = prefs.getBytes("smax", sensorMax, sizeof(sensorMax));
  lineIsDark = prefs.getBool("invert", true);
  prefs.end();
  if (r1 != sizeof(sensorMin) || r2 != sizeof(sensorMax)) {
    for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  }
}
bool saveLedColor() {
  prefs.begin("linefw", false);
  prefs.putUChar("lr", idleR);
  prefs.putUChar("lg", idleG);
  prefs.putUChar("lb", idleB);
  prefs.end();

  prefs.begin("linefw", true);
  bool ok = prefs.getUChar("lr", (uint8_t)(idleR ^ 0xFF)) == idleR &&
            prefs.getUChar("lg", (uint8_t)(idleG ^ 0xFF)) == idleG &&
            prefs.getUChar("lb", (uint8_t)(idleB ^ 0xFF)) == idleB;
  prefs.end();

  confirmSaveResult(ok);
  return ok;
}
void loadLedColor() {
  prefs.begin("linefw", true);
  idleR = prefs.getUChar("lr", 0);
  idleG = prefs.getUChar("lg", 0);
  idleB = prefs.getUChar("lb", 60);
  prefs.end();
}
bool saveSettings() {
  prefs.begin("linefw", false);
  prefs.putFloat("kp", Kp);
  prefs.putFloat("ki", Ki);
  prefs.putFloat("kd", Kd);
  prefs.putInt("base", baseSpeed);
  prefs.putInt("tspd", testSpeed);
  prefs.putBool("tpleft", turnPriorityLeft);
  prefs.putInt("utms", uTurnMinMs);
  prefs.putInt("ema", sensorEmaDen);
  prefs.putInt("dband", errDeadband);
  prefs.end();

  prefs.begin("linefw", true);
  bool ok = prefs.getFloat("kp", Kp + 1) == Kp &&
            prefs.getFloat("ki", Ki + 1) == Ki &&
            prefs.getFloat("kd", Kd + 1) == Kd &&
            prefs.getInt("base", baseSpeed + 1) == baseSpeed &&
            prefs.getInt("tspd", testSpeed + 1) == testSpeed &&
            prefs.getBool("tpleft", !turnPriorityLeft) == turnPriorityLeft &&
            prefs.getInt("utms", uTurnMinMs + 1) == uTurnMinMs &&
            prefs.getInt("ema", sensorEmaDen + 1) == sensorEmaDen &&
            prefs.getInt("dband", errDeadband + 1) == errDeadband;
  prefs.end();

  confirmSaveResult(ok);
  return ok;
}
void loadSettings() {
  prefs.begin("linefw", true);
  Kp = prefs.getFloat("kp", Kp);
  Ki = prefs.getFloat("ki", Ki);
  Kd = prefs.getFloat("kd", Kd);
  baseSpeed = prefs.getInt("base", baseSpeed);
  testSpeed = prefs.getInt("tspd", testSpeed);
  turnPriorityLeft = prefs.getBool("tpleft", turnPriorityLeft);
  uTurnMinMs = prefs.getInt("utms", uTurnMinMs);
  sensorEmaDen = prefs.getInt("ema", sensorEmaDen);
  errDeadband = prefs.getInt("dband", errDeadband);
  prefs.end();
}

// =====================================================================
//  OLED HELPERS
// =====================================================================
void oledHeader(const char* title) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
}

// One line of button-hint text, opaque background so it stays readable
// over other content.
void oledFooter(const char* hint) {
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print(hint);
  display.setTextColor(SSD1306_WHITE);
}

// =====================================================================
//  MODE: MENU
// =====================================================================
void enterState(State s) {
  currentState = s;
  applyModeLED();
  beepOK();
}

void drawMenu() {
  oledHeader("MAIN MENU");
  for (int i = 0; i < menuCount; i++) {
    display.setCursor(2, 12 + i * 9);
    display.print(i == menuIndex ? "> " : "  ");
    display.println(menuItems[i]);
  }
  oledFooter("UP/DN Move  OK Select");
  display.display();
}

void handleMenu() {
  updateIdleBreath();
  drawMenu();
  if (up())   { menuIndex = (menuIndex - 1 + menuCount) % menuCount; beep(1800, 30); }
  if (down()) { menuIndex = (menuIndex + 1) % menuCount; beep(1800, 30); }
  if (select()) {
    switch (menuIndex) {
      case 0: enterState(STATE_LINE_FOLLOW); lineFollowStarting = true; break;
      case 1: enterState(STATE_CALIBRATION); calibPhase = CALIB_SELECT_MODE; break;
      case 2: enterState(STATE_MOTOR_TEST); break;
      case 3: enterState(STATE_LED_COLOR); break;
      case 4: enterState(STATE_PID_TUNE); break;
      case 5: enterState(STATE_SENSOR_TEST); break;
    }
  }
}

// =====================================================================
//  MODE: SENSOR TEST  (built-in hardware diagnostic for the 14 IR sensors)
// =====================================================================
// Proves the sensor front end works BEFORE trusting line following, and
// specifically lets you SEE whether the noise fixes above are working.
// Per sensor it answers:
//   1. Alive?      -> RAW moves when you pass the line under it
//   2. Quiet?      -> NOISE (peak-to-peak) is small, page 3
//   3. Calibrated? -> SPAN (max-min) is wide, page 3
//   4. Sees line?  -> the on/off cap and count match reality
// Motors are never driven here. OK cycles pages, UP flips polarity,
// BACK exits.
int sensorTestPage = 0;

void sensorTestLoop() {
  readAllSensors();
  normalizeSensors();

  if (select()) { sensorTestPage = (sensorTestPage + 1) % 3; beep(1900, 25); }
  if (up())     { lineIsDark = !lineIsDark; beep(1500, 30); }
  if (back())   { beepBack(); enterState(STATE_MENU); return; }

  int onCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) if (sensorIsOn(i)) onCount++;

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw < 60) return; // ADC reads are cheap, the I2C push is not
  lastDraw = millis();

  // ---------- Page 1: live bars ----------
  if (sensorTestPage == 0) {
    oledHeader("SENSORS 1/3 BARS");
    const int x0 = 2, barTop = 18, barH = 28, colW = 9;
    for (int i = 0; i < NUM_SENSORS; i++) {
      int w = lineWeight(i);
      int h = map(w, 0, 1000, 0, barH);
      int x = x0 + i * colW;
      display.drawRect(x, barTop, colW - 2, barH, SSD1306_WHITE);
      if (h > 0) display.fillRect(x, barTop + (barH - h), colW - 2, h, SSD1306_WHITE);
      if (sensorIsOn(i)) display.fillRect(x, barTop - 4, colW - 2, 3, SSD1306_WHITE);
    }
    display.setCursor(0, 50);
    display.print("On:"); display.print(onCount); display.print("/14 ");
    display.print(onCount == 0 ? "NO LINE" : (onCount >= 12 ? "ENDZONE?" : "OK"));
    oledFooter("OK Page  UP Polarity");
    display.display();
    return;
  }

  // ---------- Page 2: numbers ----------
  if (sensorTestPage == 1) {
    oledHeader("SENSORS 2/3 RAW");
    for (int i = 0; i < NUM_SENSORS; i++) {
      int col = (i < 7) ? 0 : 64;
      int row = 14 + (i % 7) * 7;
      display.setCursor(col, row);
      if (i < 10) display.print(" ");
      display.print(i);
      display.print(sensorIsOn(i) ? "*" : " ");
      display.print(sensorRaw[i]);
      display.print("/");
      display.print(lineWeight(i));
    }
    oledFooter("OK Page  BACK Menu");
    display.display();
    return;
  }

  // ---------- Page 3: health -- finds broken/noisy hardware ----------
  // NOISE is the number to watch for the fluctuation problem: it is the
  // peak-to-peak spread across the samples taken this cycle, BEFORE the
  // median picks one. Quiet wiring reads single digits. Tens or hundreds
  // means the noise is entering the analog path in HARDWARE (shared
  // ground with the motor rail, unshielded sensor leads, weak supply) and
  // no amount of software filtering fully hides it.
  oledHeader("SENSORS 3/3 HEALTH");
  int weak = 0, firstWeak = -1, minSpan = 4096;
  int worstNoise = 0, worstNoiseIdx = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int span = sensorMax[i] - sensorMin[i];
    if (span < minSpan) minSpan = span;
    if (span < 200) { weak++; if (firstWeak < 0) firstWeak = i; }
    if (sensorNoise[i] > worstNoise) { worstNoise = sensorNoise[i]; worstNoiseIdx = i; }
  }
  display.setCursor(0, 14);
  display.print("Noise max:"); display.print(worstNoise);
  display.print(" S"); display.println(worstNoiseIdx);
  display.setCursor(0, 24);
  display.print("  ");
  display.println(worstNoise <= 10 ? "QUIET - good" :
                  worstNoise <= 40 ? "some noise" : "NOISY-check wiring");
  display.setCursor(0, 34);
  display.print("Span min:"); display.println(minSpan);
  display.setCursor(0, 44);
  if (weak == 0) display.println("All 14 calibrated");
  else { display.print("WEAK:"); display.print(weak); display.print(" 1st=S"); display.println(firstWeak); }
  oledFooter("OK Page  UP Polarity");
  display.display();

  Serial.print("SENS on="); Serial.print(onCount);
  Serial.print(" noiseMax="); Serial.print(worstNoise);
  Serial.print(" | ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(i); Serial.print(":"); Serial.print(sensorRaw[i]);
    Serial.print("/"); Serial.print(lineWeight(i));
    Serial.print("~"); Serial.print(sensorNoise[i]);
    Serial.print(sensorIsOn(i) ? "* " : "  ");
  }
  Serial.println();
}

// =====================================================================
//  MODE: CALIBRATION
// =====================================================================
void calibrationLoop() {
  // ---- Step 1: pick MANUAL (hand-sweep) or AUTO (360 deg motor spin) ----
  if (calibPhase == CALIB_SELECT_MODE) {
    oledHeader("CALIBRATE");
    display.setCursor(0, 14);
    display.println("Choose mode:");
    display.setCursor(4, 28);
    display.print(calibMode == CALIB_MANUAL ? "> " : "  ");
    display.println("MANUAL (hand sweep)");
    display.setCursor(4, 38);
    display.print(calibMode == CALIB_AUTO ? "> " : "  ");
    display.println("AUTO (360 spin)");
    oledFooter("UD=Pick OK=Go BK=Back");
    display.display();

    if (up() || down()) { calibMode = (calibMode == CALIB_MANUAL) ? CALIB_AUTO : CALIB_MANUAL; beep(1800, 20); }
    if (select()) {
      for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
      calibPhase = CALIB_RUNNING;
      if (calibMode == CALIB_AUTO) { calibAutoStart = millis(); beepStart(); }
      else beep(1800, 30);
    }
    if (back()) { calibPhase = CALIB_SELECT_MODE; beepBack(); enterState(STATE_MENU); }
    return;
  }

  // ---- Step 2: AUTO spins in place for CALIB_AUTO_DURATION_MS, then stops
  // and hands off to the same review/save flow MANUAL always used. ----
  bool spinning = (calibMode == CALIB_AUTO && calibPhase == CALIB_RUNNING);
  if (spinning) {
    if (millis() - calibAutoStart < CALIB_AUTO_DURATION_MS) {
      setLeftMotor(CALIB_AUTO_SPEED);
      setRightMotor(-CALIB_AUTO_SPEED);
    } else {
      stopMotors();
      calibPhase = CALIB_AUTO_DONE;
      spinning = false;
      beepStart();
    }
  }

  // Sensor read + min/max tracking run every loop (cheap) so calibration
  // never misses a peak, even though the display below is throttled.
  readAllSensors();
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
    if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
  }

  if (up()) { lineIsDark = !lineIsDark; beep(1700, 20); }
  normalizeSensors(); // needed live (not just at Save) so the on-line tick row below is accurate

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 40) {
    lastDraw = millis();

    // Overall contrast quality: average (max-min) spread across all
    // sensors. Low spread means the sensor barely sees a difference
    // between line and background -- height, ambient light, or a sensor
    // that never got waved over the line will all show up as LOW here,
    // instead of you having to guess whether you calibrated "enough".
    long totalSpread = 0;
    for (int i = 0; i < NUM_SENSORS; i++) totalSpread += (sensorMax[i] - sensorMin[i]);
    bool goodContrast = (totalSpread / NUM_SENSORS) > 800; // out of a 0-4095 ADC range

    oledHeader(spinning ? "CALIB (spinning)" : "CALIB");
    display.setCursor(96, 0);
    display.print(lineIsDark ? "DRK" : "LIT");

    // ---- per-sensor bar visualizer (one bar per sensor, live) ----
    const int tickY      = 11;            // 1px row: which sensors read "on line" right now
    const int barTop     = 13;
    const int barBottom  = OLED_H;        // 64
    const int barMaxH    = barBottom - barTop;
    int slot = OLED_W / NUM_SENSORS;      // ~9px per sensor on a 128-wide display
    if (slot < 6) slot = 6;

    for (int i = 0; i < NUM_SENSORS; i++) {
      int lo = sensorMin[i], hi = sensorMax[i];
      int h = 0;
      if (hi > lo) {
        int v = constrain(sensorRaw[i], lo, hi);
        h = map(v, lo, hi, 0, barMaxH);
      }
      int x = i * slot;
      int w = slot - 2;
      if (w < 2) w = 2;
      display.drawRect(x, barTop, w, barMaxH, SSD1306_WHITE);          // outline
      display.fillRect(x + 1, barBottom - h, max(w - 2, 1), h, SSD1306_WHITE); // fill from bottom
      // Live proof the DARK/LITE setting is doing what you think: a tick
      // above any sensor Line Follow would currently treat as "on line".
      if (lineWeight(i) > ON_THRESH) display.fillRect(x, tickY, w, 1, SSD1306_WHITE);
    }

    if (spinning) {
      oledFooter("Spinning.. BK=Stop");
    } else {
      display.setCursor(0, 48);
      display.print("Contrast: "); display.println(goodContrast ? "OK" : "LOW");
      oledFooter("OK=Save BK=Cancel UP=Inv");
    }
    display.display();
  }

  if (select()) { // Save & exit — saveCalibration() itself beeps/flashes
    // the verified pass/fail result, so no extra beepStart() here.
    stopMotors(); // in case OK is pressed mid-spin to stop and save early
    saveCalibration();
    calibPhase = CALIB_SELECT_MODE;
    enterState(STATE_MENU);
  }
  if (back()) { // Cancel without saving (also stops an in-progress spin)
    stopMotors();
    loadCalibration();
    calibPhase = CALIB_SELECT_MODE;
    beepBack();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  MODE: MOTOR TEST
// =====================================================================
// OK    = cycle target (Left / Right / Both)
// UP    = hold to drive target forward
// DOWN  = hold to drive target reverse
// BACK  = stop & return to menu
//
// Forward and reverse share one code path (dir * testSpeed fed to the
// same setLeftMotor()/setRightMotor() calls for both signs), so there's
// no separate "reverse" branch to get out of sync between motors.
// testSpeed lives up in CONSTANTS (needed there by saveSettings/loadSettings)
MotorTestTarget motorTestTarget = MT_BOTH;
const char* motorTestTargetName[3] = {"LEFT", "RIGHT", "BOTH"};

void motorTestLoop() {
  updateEncoderRates();

  if (select()) {
    motorTestTarget = (MotorTestTarget)((motorTestTarget + 1) % 3);
    beep(2200, 20);
  }

  bool fwdHeld = upHeld();
  bool revHeld = downHeld();
  int dir = fwdHeld ? 1 : (revHeld ? -1 : 0);

  static bool wasDriving = false;
  bool driving = (dir != 0);
  if (driving && !wasDriving) beep(dir > 0 ? 1300 : 700, 40); // direction cue on press
  wasDriving = driving;

  int cmd = dir * testSpeed;
  switch (motorTestTarget) {
    case MT_LEFT:  setLeftMotor(cmd); setRightMotor(0);   break;
    case MT_RIGHT: setLeftMotor(0);   setRightMotor(cmd); break;
    case MT_BOTH:  setLeftMotor(cmd); setRightMotor(cmd); break;
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 50) {
    lastDraw = millis();

    oledHeader("MOTOR TEST");
    // All three targets always shown, current one bracketed — reachable and
    // visible on-screen regardless of whether the buzzer/LED cue is heard.
    display.setCursor(0, 12);
    display.print(motorTestTarget == MT_LEFT  ? "[L] " : " L  ");
    display.print(motorTestTarget == MT_RIGHT ? "[R] " : " R  ");
    display.print(motorTestTarget == MT_BOTH  ? "[BOTH]" : " BOTH ");
    display.setCursor(0, 22);
    display.println(dir > 0 ? "-> FWD" : dir < 0 ? "<- REV" : "-- STOP");

    display.setCursor(0, 34);
    display.print("L "); display.print(leftEncCount); display.print("  ");
    if (ENCODER_COUNTS_PER_REV > 0) { display.print(leftRPM, 0); display.println("rpm"); }
    else { display.print(leftCPS, 0); display.println("cps"); }

    display.setCursor(0, 44);
    display.print("R "); display.print(rightEncCount); display.print("  ");
    if (ENCODER_COUNTS_PER_REV > 0) { display.print(rightRPM, 0); display.println("rpm"); }
    else { display.print(rightCPS, 0); display.println("cps"); }

    oledFooter("OK=Target UP/DN=Drive");
    display.display();
  }

  if (back()) {
    stopMotors();
    beepBack();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  MODE: LINE FOLLOW
// =====================================================================
float pidIntegral = 0;
float pidLastError = 0;

unsigned long lastLineLostBeep = 0;

// positions centered around 0, spaced 1000 apart across 14 sensors
long sensorPosition(int i) {
  return (long)i * 1000L - 6500L;
}

const char* directionArrow(int left, int right) {
  if (left > right + 10) return "<<<";
  if (right > left + 10) return ">>>";
  if (left > 0)          return "^^^";
  if (left < 0)           return "vvv";
  return "---";
}

int lastEdgeDir = 0;            // -1=left, 1=right, 0=neither seen yet -- sticky across the run
unsigned long lineLostSince = 0; // 0 = currently on the line
bool endZoneReached = false;
unsigned long endZoneSince = 0;  // 0 = not currently seeing a wide block

// ---- Checkpoint U-turn: a dead end with NO edge-sensor evidence on either
// side (as opposed to a real branch, where an edge lit up recently) means
// the track genuinely stops here -- per the arena, that's a checkpoint
// marker and the correct move is a controlled 180, not a guessed corner
// turn.
//
// uTurnMinMs is the important one: the checkpoint's own crossing mark
// sweeps under the sensors almost immediately once the pivot starts, so
// checking for "line reacquired" from the very first instant made this
// stop after a few degrees instead of completing the turn -- exactly the
// bug reported ("unable to take a U-turn... as it should"). The STM
// reference's TURN_AROUND phase sidesteps this entirely: it's a BLIND,
// fixed-duration pivot that ignores the sensors completely until the
// phase timer is up. Same fix here: sensor reacquisition is ignored for
// the first uTurnMinMs no matter what the checkpoint mark does, only
// checked after that. uTurnMinMs is a guess (600ms @ UTURN_SPEED) --
// tune it on the real arena until the pivot actually reads as ~180
// degrees; it's the single most important constant in this file.
bool uTurning = false;
unsigned long uTurnStart = 0;
// uTurnMinMs lives up with the other tunables (it's adjustable from
// Settings as "UturnMs" -- it needs on-arena calibration to land a true
// 180, so it shouldn't require a reflash to change).
const unsigned long UTURN_MAX_MS = 2500; // safety cap if the line is never seen again
const int UTURN_SPEED = 130;

void lineFollowLoop() {
  if (lineFollowStarting) {
    pidIntegral = 0;
    pidLastError = 0;
    lastEdgeDir = 0;
    lineLostSince = 0;
    endZoneReached = false;
    endZoneSince = 0;
    beepStart();
    lineFollowStarting = false;
  }

  // Run's over -- just show the result and wait for BACK. No PID, no
  // motor drive (already stopped at the moment End Zone was confirmed).
  if (endZoneReached) {
    oledHeader("END ZONE!");
    display.setCursor(0, 24);
    display.println("Run complete.");
    oledFooter("BACK = Menu");
    display.display();
    if (back()) { setLED({0, 0, 0}); enterState(STATE_MENU); }
    return;
  }

  readAllSensors();
  normalizeSensors();

  // Mid-U-turn: blind pivot for the first uTurnMinMs (sensors ignored
  // completely, same as the STM reference's fixed-duration TURN_AROUND),
  // THEN look for the line reappearing under any sensor, up to UTURN_MAX_MS
  // as a safety net. This bypasses the PID/error path entirely -- it's a
  // fixed maneuver, not a steered one.
  if (uTurning) {
    unsigned long uElapsed = millis() - uTurnStart;
    bool reacquired = false;
    if (uElapsed >= uTurnMinMs) {
      long checkSum = 0;
      for (int i = 0; i < NUM_SENSORS; i++) checkSum += lineWeight(i);
      reacquired = checkSum > (long)(NUM_SENSORS * 30);
    }
    if (reacquired || uElapsed > UTURN_MAX_MS) {
      uTurning = false;
      lineLostSince = 0;
      pidLastError = 0; // avoid a derivative kick from whatever error existed before the turn
    } else {
      setLeftMotor(-UTURN_SPEED);
      setRightMotor(UTURN_SPEED);
      updateLineFollowLED(false);
      static unsigned long lastUDraw = 0;
      if (millis() - lastUDraw >= 50) {
        lastUDraw = millis();
        oledHeader("LINE FOLLOWING");
        display.setCursor(0, 24);
        display.println(uElapsed < uTurnMinMs ? "U-TURN (blind)" : "U-TURN (checking)");
        oledFooter("BACK = Stop");
        display.display();
      }
      if (back()) { stopMotors(); uTurning = false; beepBack(); enterState(STATE_MENU); }
      return;
    }
  }

  // weightedSum/rawSum use the true sensor weight for detection & End Zone
  // sizing; gainedSum applies EDGE_GAIN on top for the error CALCULATION
  // only, so edge sensors steer harder without lying about how many
  // sensors are actually lit (which End Zone detection depends on).
  long weightedSum = 0, rawSum = 0, gainedSum = 0;
  int onCount = 0;
  bool leftEdgeNow = false, rightEdgeNow = false, centerFound = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int w = lineWeight(i);
    rawSum += w;
    bool edge = (i < EDGE_COUNT) || (i >= NUM_SENSORS - EDGE_COUNT);
    long gw = edge ? (long)(w * EDGE_GAIN) : (long)w;
    weightedSum += gw * sensorPosition(i);
    gainedSum += gw;
    if (sensorIsOn(i)) { // hysteresis, not a bare compare -- see ON_HYST
      onCount++;
      if (edge) { if (i < EDGE_COUNT) leftEdgeNow = true; else rightEdgeNow = true; }
      else centerFound = true;
    }
  }
  // Update the sticky turn-direction memory from this cycle's edge reads.
  // When both sides are active at once, turnPriorityLeft (a Settings
  // option, not a hardcoded side) decides the tie -- previously this fell
  // out of loop-iteration order (right silently won by being processed
  // last). Only one side active always wins regardless of the setting;
  // neither active this cycle leaves lastEdgeDir untouched (sticky).
  if (leftEdgeNow && rightEdgeNow)  lastEdgeDir = turnPriorityLeft ? -1 : 1;
  else if (leftEdgeNow)             lastEdgeDir = -1;
  else if (rightEdgeNow)            lastEdgeDir = 1;

  // ---- End Zone: sustained wide-black reading, not just a junction blip ----
  if (onCount >= END_ZONE_MIN_SENSORS) {
    if (endZoneSince == 0) endZoneSince = millis();
    if (millis() - endZoneSince >= END_ZONE_HOLD_MS) {
      stopMotors();
      endZoneReached = true;
      setLED({255, 255, 255}); // only now -- never mid-run
      beep(2200, 120); delay(140);
      beep(2600, 120); delay(140);
      beep(3000, 200);
      return;
    }
  } else {
    endZoneSince = 0;
  }

  bool lineFound = rawSum > (long)(NUM_SENSORS * 30); // "anything at all" -- still used for
                                                        // integral gating, LED, and the display
  float error;
  bool tracking; // true only when center sees the line -> smooth PID, integral allowed

  if (centerFound) {
    // Mid line is there: just go straight (well, follow it) -- no turn
    // decision needed at all. Full weighted average (edge-gained) as usual.
    tracking = true;
    error = (float)weightedSum / (float)gainedSum;
    lineLostSince = 0;
  } else if (leftEdgeNow || rightEdgeNow) {
    // Mid line is GONE but an edge sees it RIGHT NOW: commit that way
    // immediately -- don't wait for the whole array to go blank first.
    tracking = false;
    if (leftEdgeNow && rightEdgeNow) error = turnPriorityLeft ? -6500 : 6500;
    else                              error = leftEdgeNow ? -6500 : 6500;
    lineLostSince = 0;
    if (millis() - lastLineLostBeep > 500) { beepError(); lastLineLostBeep = millis(); }
  } else {
    // Nothing anywhere -- center AND both edges all blank. Coast briefly
    // (covers a genuine momentary gap), then it's a dead end -> U-turn.
    tracking = false;
    if (lineLostSince == 0) lineLostSince = millis();
    unsigned long lostFor = millis() - lineLostSince;
    if (lostFor < LINE_LOST_COAST_MS) {
      error = 0;
    } else {
      uTurning = true;
      uTurnStart = millis();
      beep(1000, 100);
      setLeftMotor(-UTURN_SPEED);
      setRightMotor(UTURN_SPEED);
      return;
    }
  }

  // ---- Error conditioning: this is what the wheels actually feel ----
  // Even with clean sensors, the weighted-average error twitches by a few
  // counts every cycle, and Kd multiplies exactly that twitch. Two steps:
  //   DEADBAND -- inside a few counts of centre, call it centre. Stops the
  //     bot hunting left-right while it is already dead on the line.
  //   SMOOTHING -- a light low-pass on the error itself, so a single noisy
  //     cycle cannot spike the derivative term.
  // Both apply only while tracking; a committed turn must stay sharp.
  if (tracking) {
    if (error > -errDeadband && error < errDeadband) error = 0;
    errorFiltered = (errorFiltered * (ERR_EMA_DEN - 1) + error) / ERR_EMA_DEN;
    error = errorFiltered;
  } else {
    errorFiltered = error; // re-seed so tracking resumes from the real value
  }

  // Only accumulate the integral term while smoothly tracking the center --
  // a committed hard turn or a coast has `error` pinned to a saturated or
  // zero value that would either slam the integral into its clamp or just
  // waste the accumulated history; freeze it outside normal tracking.
  if (tracking) {
    pidIntegral += error;
    pidIntegral = constrain(pidIntegral, -50000, 50000);
  }
  float derivative = error - pidLastError;
  float output = Kp * error + Ki * pidIntegral + Kd * derivative;
  pidLastError = error;

  int left  = baseSpeed - (int)output;
  int right = baseSpeed + (int)output;
  setLeftMotor(constrain(left, -255, 255));
  setRightMotor(constrain(right, -255, 255));

  updateLineFollowLED(lineFound);

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 50) { // ~20 Hz screen refresh, decoupled from the PID loop above
    lastDraw = millis();

    oledHeader("LINE FOLLOWING");
    display.setCursor(0, 14);
    display.print("Err: "); display.println((int)error);
    display.setCursor(0, 24);
    display.print("L:"); display.print(left);
    display.print("  R:"); display.print(right);
    display.setCursor(90, 24);
    display.print(directionArrow(left, right));
    display.setCursor(0, 40);
    display.println(tracking ? "Mid: OK" : (leftEdgeNow || rightEdgeNow) ? (leftEdgeNow ? "Turn: LEFT" : "Turn: RIGHT") : "Mid: LOST (coast)");
    oledFooter("BACK = Stop");
    display.display();

#if DEBUG_SERIAL
    Serial.print("err="); Serial.print(error);
    Serial.print(" L="); Serial.print(left);
    Serial.print(" R="); Serial.print(right);
    Serial.print(" on="); Serial.print(onCount);
    Serial.print(" edgeDir="); Serial.print(lastEdgeDir);
    Serial.print(" found="); Serial.println(lineFound ? 1 : 0);
#endif
  }

  if (back()) {
    stopMotors();
    beepBack();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  MODE: LED COLOR PICKER
// =====================================================================
//   UP   = -10 on selected channel
//   DOWN = +10 on selected channel
//   OK   = switch channel (R/G/B)
//   BACK = save & back
int ledChannel = 0; // 0=R,1=G,2=B,3=PRESET
const char* ledChannelName[4] = {"R", "G", "B", "PRESET"};

// Named preset colors -- cycle to the 4th "channel" (PRESET) with OK, then
// UP/DN pick a color and apply it instantly, instead of hand-tuning R/G/B.
struct NamedColor { const char* name; uint8_t r, g, b; };
const NamedColor ledPresets[] = {
  {"RED", 255, 0, 0}, {"GREEN", 0, 255, 0}, {"BLUE", 0, 0, 255},
  {"WHITE", 255, 255, 255}, {"CYAN", 0, 255, 255}, {"MAGENTA", 255, 0, 255},
  {"YELLOW", 255, 200, 0}, {"ORANGE", 255, 80, 0}, {"PURPLE", 130, 0, 200}
};
const int ledPresetCount = sizeof(ledPresets) / sizeof(ledPresets[0]);
int presetIndex = 0;

void applyLedPreset() {
  idleR = ledPresets[presetIndex].r;
  idleG = ledPresets[presetIndex].g;
  idleB = ledPresets[presetIndex].b;
}

void ledColorLoop() {
  oledHeader("LED COLOR");
  display.setCursor(0, 14);
  display.print("Channel: "); display.println(ledChannelName[ledChannel]);

  if (ledChannel == 3) {
    display.setCursor(0, 24);
    display.print("Preset: "); display.println(ledPresets[presetIndex].name);
    oledFooter("UP/DN=Pick OK=Next");
  } else {
    display.setCursor(0, 24);
    display.print("R:"); display.print(idleR);
    display.print(" G:"); display.print(idleG);
    display.print(" B:"); display.println(idleB);
    oledFooter("UP=- DN=+ OK=Next");
  }
  display.setCursor(0, 46);
  display.println("BACK=Save & Back");
  display.display();

  if (ledChannel == 3) {
    if (up())   { presetIndex = (presetIndex - 1 + ledPresetCount) % ledPresetCount; applyLedPreset(); beep(1600,20); }
    if (down()) { presetIndex = (presetIndex + 1) % ledPresetCount; applyLedPreset(); beep(2000,20); }
  } else {
    uint8_t* chan = (ledChannel == 0) ? &idleR : (ledChannel == 1) ? &idleG : &idleB;
    if (up())   { *chan = (uint8_t)max(0, *chan - 10); beep(1600,20); }
    if (down()) { *chan = (uint8_t)min(255, *chan + 10); beep(2000,20); }
  }
  if (select()) { ledChannel = (ledChannel + 1) % 4; beep(2200,20); }

  setLED({idleR, idleG, idleB}); // live preview

  if (back()) { // saveLedColor() beeps/flashes the verified result itself
    saveLedColor();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  MODE: SETTINGS  (was "PID Tune"; now also holds the motor-test speed)
// =====================================================================
//   UP   = cycle parameter (Kp -> Ki -> Kd -> Base Speed -> Test Speed)
//   DOWN = decrease
//   OK   = increase
//   BACK = save & back
int pidParamIndex = 0; // 0=Kp,1=Ki,2=Kd,3=BaseSpeed,4=TestSpeed,5=TurnPriority,6=UturnMs
const char* settingsParamName[9] = {"Kp", "Ki", "Kd", "BaseSpd", "TestSpd", "TurnPri", "UturnMs", "Smooth", "Deadband"};
const int settingsParamCount = 9;

void settingsLoop() {
  oledHeader("SETTINGS");
  display.setCursor(0, 14);
  display.print("Param: "); display.println(settingsParamName[pidParamIndex]);

  // Big readout of just the selected value — stays legible and scales to
  // more parameters later without running out of vertical space on a
  // 64px-tall screen.
  display.setTextSize(2);
  display.setCursor(0, 28);
  switch (pidParamIndex) {
    case 0: display.println(Kp, 3);        break;
    case 1: display.println(Ki, 4);        break;
    case 2: display.println(Kd, 2);        break;
    case 3: display.println(baseSpeed);    break;
    case 4: display.println(testSpeed);    break;
    case 5: display.println(turnPriorityLeft ? "LEFT" : "RIGHT"); break;
    case 6: display.println(uTurnMinMs);   break;
    case 7: display.println(sensorEmaDen); break;
    case 8: display.println(errDeadband);  break;
  }
  display.setTextSize(1);

  oledFooter(pidParamIndex == 5 ? "UP=Param DN/OK=Toggle" : "UP=Param DN/OK=-/+ BK=Sav");
  display.display();

  if (up()) { pidParamIndex = (pidParamIndex + 1) % settingsParamCount; beep(1800,20); }

  bool dec = down();
  bool inc = select();
  if (dec || inc) {
    int dir = inc ? 1 : -1;
    switch (pidParamIndex) {
      case 0: Kp += dir * 0.005f; Kp = max(0.0f, Kp); break;
      case 1: Ki += dir * 0.0001f; Ki = max(0.0f, Ki); break;
      case 2: Kd += dir * 0.02f; Kd = max(0.0f, Kd); break;
      case 3: baseSpeed += dir * 25; baseSpeed = constrain(baseSpeed, 0, 255); break;
      case 4: testSpeed += dir * 25; testSpeed = constrain(testSpeed, 0, 255); break;
      case 5: turnPriorityLeft = !turnPriorityLeft; break; // binary -- either DN or OK just flips it
      case 6: uTurnMinMs += dir * 50; uTurnMinMs = constrain(uTurnMinMs, 0, 2000); break;
      case 7: sensorEmaDen += dir; sensorEmaDen = constrain(sensorEmaDen, 1, 16); break;
      case 8: errDeadband += dir * 25; errDeadband = constrain(errDeadband, 0, 1000); break;
    }
    beep(2000, 20);
  }

  if (back()) { // saveSettings() beeps/flashes the verified result itself
    saveSettings();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

  // Buzzer + LED are brought up and exercised FIRST, before anything that
  // could stall setup() (I2C/OLED, motor PWM). If you don't see a white
  // flash and hear a beep right here at power-on, it's wiring/power on
  // those two peripherals, not a bug further down in the state machine —
  // see BUZZER_PIN gets its own explicit LEDC channel (separate from the
  // 4 motor channels) so beep() below can't be starved by motorsInit().
  pinMode(BUZZER_PIN, OUTPUT);
  ledcAttach(BUZZER_PIN, 2000, 8);
  strip.begin();
  strip.setBrightness(80);
  strip.show();
  setLED({255, 255, 255});
  beep(1800, 120);
  delay(150);
  setLED({0, 0, 0});
  buzzerService();
  delay(50);

  // Mux pins
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  analogReadResolution(12); // 0-4095
  // Full 0-3.3V range on the mux output. Without this the default
  // attenuation clips the top of the range, compressing the usable span
  // and making the calibrated min/max window narrower than it should be
  // -- another thing that reads as "noise" once normalized.
  analogSetPinAttenuation(MUX_SIG, ADC_11db);

  // Buttons (external or internal pull-ups assumed; switches to GND)
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);

  // Encoders
  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);

  // Motors
  motorsInit();
  stopMotors();

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000); // fast-mode I2C: keeps display refresh from stealing loop time
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // OLED failed - blink LED red as an error indicator, halt is avoided
    // so the robot doesn't become unusable; just flag on screen retries.
    for (int i = 0; i < 3; i++) {
      setLED({255,0,0}); delay(150);
      setLED({0,0,0}); delay(150);
    }
  }
  display.clearDisplay();
  display.display();

  // Load saved settings
  loadCalibration();
  loadLedColor();
  loadSettings();

  applyModeLED();
  beepOK();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  updateEncoderRates();
  buzzerService();

  switch (currentState) {
    case STATE_MENU:        handleMenu();      break;
    case STATE_CALIBRATION: calibrationLoop(); break;
    case STATE_MOTOR_TEST:  motorTestLoop();   break;
    case STATE_LINE_FOLLOW: lineFollowLoop();  break;
    case STATE_LED_COLOR:   ledColorLoop();    break;
    case STATE_PID_TUNE:    settingsLoop();    break;
    case STATE_SENSOR_TEST: sensorTestLoop(); break;
  }
  delay(5); // small yield
}
