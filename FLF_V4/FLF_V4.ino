            /*
  =====================================================================
  LINE FOLLOWER ROBOT — Waveshare ESP32-S3-Pico
  =====================================================================
  Features:
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
    - Junction handling: when many sensors trigger at once (a cross/T/wide
      marker) or the line vanishes, the bot does NOT guess a fixed
      direction. It creeps straight a bounded distance first (most gaps
      and crossings resolve themselves that way), and only if that fails
      does it try Straight (already done) -> Left -> Right -> U-turn, each
      attempt bounded by actual encoder ticks (not a timer) and cancelled
      the instant the line is seen again. See NAV_* in MODE: LINE FOLLOW.
    - Corner-aware speed: cruise speed backs off automatically the harder
      the PID is correcting, and both wheels ramp toward their target
      speed instead of jumping — keeps traction at higher baseSpeed
    - Calibration shows a live contrast quality readout (OK/LOW) and a
      row of tick marks showing which sensors currently read "on line"
      under the active DARK/LITE setting, so calibration quality and
      polarity are both visible before you save, not guessed at
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
  STATE_PID_TUNE
};

enum MotorTestTarget { MT_LEFT, MT_RIGHT, MT_BOTH };

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

// ---- Line-lost navigation (see NAV_* in MODE: LINE FOLLOW) ----
const int ON_THRESH = 500;        // per-sensor lineWeight() above this = "on line";
                                   // used for calibration's live tick row and to
                                   // decide which side has a branch during a search
const int NAV_TURN_SPEED = 110;   // reduced, traction-friendly speed while searching

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
bool calibResetFlag = false;
bool lineFollowStarting = false;

// Menu
const char* menuItems[] = {
  "Calibrate Sensors",
  "Test Motors",
  "Start Line Follow",
  "LED Color",
  "Settings"
};
const int menuCount = 5;
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
// Durations/frequencies picked to be audible over motor/gearbox noise —
// small piezo buzzers are typically loudest in the 2.5-3.5kHz range, and
// longer pulses read as noticeably louder than short clicks even at the
// same drive level.
void beepOK()    { beep(2600, 90); }
void beepBack()  { beep(900, 120); }
void beepError() { beep(500, 350); }
void beepStart() { beep(1500, 110); delay(120); beep(2600, 160); }

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
// Physical wiring: SW1, SW2, SW3, SW4. Current layout rotates the roles
// one step around the enclosure: SW1->BACK, SW2->UP, SW3->OK (unchanged),
// SW4->DOWN. Every screen below calls up()/down()/select()/back() by
// role, never by SWx, so the physical layout can change again by editing
// only this block.
bool up()     { return wasPressed(btn2); }  // physical SW2
bool down()   { return wasPressed(btn4); }  // physical SW4
bool select() { return wasPressed(btn3); }  // physical SW3
bool back()   { return wasPressed(btn1); }  // physical SW1

// Raw (non-edge-triggered) hold state for the same UP/DOWN roles, used by
// Motor Test's hold-to-drive controls.
bool upHeld()   { return digitalRead(SW2) == LOW; }
bool downHeld() { return digitalRead(SW4) == LOW; }

// =====================================================================
//  MULTIPLEXER / SENSOR READING
// =====================================================================
void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
  delayMicroseconds(8); // let mux + sensor line settle
}

void readAllSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(muxChannelForSensor[i]);
    sensorRaw[i] = analogRead(MUX_SIG); // 0-4095 on ESP32 ADC
  }
}

void normalizeSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int lo = sensorMin[i], hi = sensorMax[i];
    if (hi <= lo) { sensorNorm[i] = 0; continue; }
    int v = constrain(sensorRaw[i], lo, hi);
    sensorNorm[i] = map(v, lo, hi, 0, 1000);
  }
}

// "How much this sensor thinks it's on the line", 0-1000, correcting for
// wiring polarity via lineIsDark so the PID math always weights toward
// the line regardless of which way the calibrated min/max fell.
int lineWeight(int i) {
  return lineIsDark ? sensorNorm[i] : (1000 - sensorNorm[i]);
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
  prefs.end();

  prefs.begin("linefw", true);
  bool ok = prefs.getFloat("kp", Kp + 1) == Kp &&
            prefs.getFloat("ki", Ki + 1) == Ki &&
            prefs.getFloat("kd", Kd + 1) == Kd &&
            prefs.getInt("base", baseSpeed + 1) == baseSpeed &&
            prefs.getInt("tspd", testSpeed + 1) == testSpeed;
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
      case 0: enterState(STATE_CALIBRATION); calibResetFlag = true; break;
      case 1: enterState(STATE_MOTOR_TEST); break;
      case 2: enterState(STATE_LINE_FOLLOW); lineFollowStarting = true; break;
      case 3: enterState(STATE_LED_COLOR); break;
      case 4: enterState(STATE_PID_TUNE); break;
    }
  }
}

// =====================================================================
//  MODE: CALIBRATION
// =====================================================================
void calibrationLoop() {
  if (calibResetFlag) {
    for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
    calibResetFlag = false;
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

    oledHeader("CALIB");
    display.setCursor(60, 0);
    display.print(lineIsDark ? "DRK" : "LIT");
    display.setCursor(96, 0);
    display.print(goodContrast ? "OK" : "LOW");

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

    oledFooter("OK=Save BK=Cancel UP=Inv");
    display.display();
  }

  if (select()) { // Save & exit — saveCalibration() itself beeps/flashes
    // the verified pass/fail result, so no extra beepStart() here.
    saveCalibration();
    enterState(STATE_MENU);
  }
  if (back()) { // Cancel without saving
    loadCalibration();
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

// ---- Line-lost navigation ----
// Weighted-average PID is ALWAYS the one driving steering in NAV_TRACK --
// this is the proven, reliable technique for a single-sensor-row bot, and
// it is deliberately never replaced by an open-loop command while it's
// active. A wide/junction-ish sensor pattern is just fed into the same
// weighted average like any other reading (a symmetric cross naturally
// averages out near "straight" on its own); the ONLY thing that gets
// special handling is the line being genuinely gone.
//
// NAV_TRACK  : normal PID tracking (a 1-2 cycle sensor blip just holds the
//              last correction via pidLastError, doesn't freeze).
// NAV_SEARCH : entered after LOST_GRACE_MS of continuous loss. Reverse-pivot
//              search, edge-sensor + time tiered -- ported directly from
//              the referenced STM32_Enhanced_LineFollower.ino's actually-
//              working recovery branch (confirmed: it's called from loop(),
//              unlike that file's dead RECOVERY_BREAK/REVERSE/ZIGZAG state
//              machine, which references undeclared variables and isn't
//              wired up). Priority each cycle: (1) whichever edge sensor
//              saw the line MOST RECENTLY (within 500ms) pivots that way
//              immediately, full speed; (2) failing that, lastEdgeDir (the
//              last side that won #1, persists across searches this run)
//              pivots that way for the first 500ms of this loss, then eases
//              off; (3) with no evidence at all, try left 0-300ms, right
//              300-600ms, then lean forward. This DOES reverse a wheel --
//              that's a deliberate choice this time (matching what's
//              proven working in the reference), not an oversight.
// NAV_GIVEUP : the search ran for SEARCH_MAX_TICKS / SEARCH_TIME_CEILING_MS
//              with no success -- STOP completely instead of searching
//              forever (the reference has an equivalent timeout-then-stop,
//              but ours is encoder-tick backed too, not just a timer). A
//              loud alarm marks it; placing the bot back on the line
//              resumes tracking on its own, or BACK returns to menu.
const unsigned long LOST_GRACE_MS = 120;      // ignore line-loss shorter than this
const unsigned long EDGE_MEMORY_MS = 500;     // how long a recent edge-sensor hit counts as "recent"
const long SEARCH_MAX_TICKS = 110;            // give up past this many encoder ticks of searching
const unsigned long SEARCH_TIME_CEILING_MS = 1800; // backup vs wheel slip (ticks alone can't be trusted)

enum NavMode { NAV_TRACK, NAV_SEARCH, NAV_GIVEUP };
NavMode navMode = NAV_TRACK;
long navEncStart = 0;
unsigned long navPhaseStart = 0;
unsigned long lostSince = 0;
bool leftEdgeSeen = false, rightEdgeSeen = false;   // sticky, cleared only at line-follow start
unsigned long leftEdgeTime = 0, rightEdgeTime = 0;  // updated every cycle an edge sensor is hot
int lastEdgeDir = 0; // -1=left, 1=right, 0=none yet -- persists across searches this run

void lineFollowLoop() {
  if (lineFollowStarting) {
    pidIntegral = 0;
    pidLastError = 0;
    navMode = NAV_TRACK;
    lostSince = 0;
    leftEdgeSeen = false;
    rightEdgeSeen = false;
    lastEdgeDir = 0;
    beepStart();
    lineFollowStarting = false;
  }

  readAllSensors();
  normalizeSensors();

  long weightedSum = 0, sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int w = lineWeight(i);
    weightedSum += (long)w * sensorPosition(i);
    sum += w;
  }
  bool lineFound = sum > (long)(NUM_SENSORS * 30);
  float error = lineFound ? (float)weightedSum / (float)sum : pidLastError;

  // Edge-sensor memory: timestamp every cycle either outer trio is hot, so
  // NAV_SEARCH can ask "which side saw the line MOST RECENTLY" rather than
  // just "ever, at some point."
  if (lineWeight(0) > ON_THRESH || lineWeight(1) > ON_THRESH || lineWeight(2) > ON_THRESH) { leftEdgeSeen = true; leftEdgeTime = millis(); }
  if (lineWeight(NUM_SENSORS-1) > ON_THRESH || lineWeight(NUM_SENSORS-2) > ON_THRESH || lineWeight(NUM_SENSORS-3) > ON_THRESH) { rightEdgeSeen = true; rightEdgeTime = millis(); }

  if (lineFound) lostSince = 0;
  else if (lostSince == 0) lostSince = millis();
  bool sustainedLoss = !lineFound && lostSince != 0 && (millis() - lostSince > LOST_GRACE_MS);

  long encNow = (leftEncCount + rightEncCount) / 2;
  long ticksThisPhase = encNow - navEncStart;
  unsigned long msThisPhase = millis() - navPhaseStart;

  int left = 0, right = 0;

  if (navMode == NAV_TRACK) {
    if (sustainedLoss) {
      navMode = NAV_SEARCH;
      navEncStart = encNow;
      navPhaseStart = millis();
      beep(1200, 50);
    }
    // PID runs every single cycle in NAV_TRACK, loss or not: `error` above
    // already falls back to pidLastError on a momentary blip, so a 1-2
    // cycle flicker just holds the last correction instead of freezing.
    if (lineFound) {
      pidIntegral += error;
      pidIntegral = constrain(pidIntegral, -50000, 50000);
    }
    float derivative = error - pidLastError;
    float output = Kp * error + Ki * pidIntegral + Kd * derivative;
    pidLastError = error;
    // Same as V3, deliberately: baseSpeed +- output, no cruise scaling. A
    // "back off cruise the harder PID corrects" scheme was tried here and
    // it silently reversed the inner wheel on any sufficiently sharp
    // correction (cruise shrinks faster than output grows), which breaks
    // traction on ordinary curves -- not just line-lost recovery. That's
    // gone; this is the plain, proven differential.
    left  = constrain(baseSpeed - (int)output, -255, 255);
    right = constrain(baseSpeed + (int)output, -255, 255);

  } else if (navMode == NAV_SEARCH) {
    if (lineFound) {
      navMode = NAV_TRACK;
      pidLastError = error;
      lostSince = 0;
    } else if (ticksThisPhase >= SEARCH_MAX_TICKS || msThisPhase >= SEARCH_TIME_CEILING_MS) {
      navMode = NAV_GIVEUP;
      stopMotors();
      for (int i = 0; i < 4; i++) { beep(3200, 150); delay(180); } // loud, unmistakable
    } else {
      unsigned long now = millis();
      long msSinceLoss = (long)(now - lostSince);
      bool recentLeft  = leftEdgeSeen  && (now - leftEdgeTime)  < EDGE_MEMORY_MS;
      bool recentRight = rightEdgeSeen && (now - rightEdgeTime) < EDGE_MEMORY_MS;

      if (recentLeft && !recentRight) {
        left = -NAV_TURN_SPEED; right = NAV_TURN_SPEED; lastEdgeDir = -1;
      } else if (recentRight && !recentLeft) {
        left = NAV_TURN_SPEED; right = -NAV_TURN_SPEED; lastEdgeDir = 1;
      } else if (lastEdgeDir == -1) {
        if (msSinceLoss < 500) { left = -NAV_TURN_SPEED; right = NAV_TURN_SPEED; }
        else                   { left = NAV_TURN_SPEED / 3; right = NAV_TURN_SPEED; }
      } else if (lastEdgeDir == 1) {
        if (msSinceLoss < 500) { left = NAV_TURN_SPEED; right = -NAV_TURN_SPEED; }
        else                   { left = NAV_TURN_SPEED; right = NAV_TURN_SPEED / 3; }
      } else { // no evidence at all yet -- brief zigzag, then lean forward
        if      (msSinceLoss < 300) { left = -NAV_TURN_SPEED; right = NAV_TURN_SPEED; }
        else if (msSinceLoss < 600) { left = NAV_TURN_SPEED; right = -NAV_TURN_SPEED; }
        else                        { left = NAV_TURN_SPEED; right = NAV_TURN_SPEED / 3; }
      }
    }

  } else { // NAV_GIVEUP -- stopped dead, no motion, until repositioned or BACK
    left = right = 0;
    if (lineFound) { // manually placed back on the line -- resume on its own
      navMode = NAV_TRACK;
      pidLastError = error;
      lostSince = 0;
      beepStart();
    }
  }

  setLeftMotor(left);
  setRightMotor(right);

  if (sustainedLoss && millis() - lastLineLostBeep > 500) { beepError(); lastLineLostBeep = millis(); }
  updateLineFollowLED(navMode == NAV_TRACK && lineFound);

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 50) { // ~20 Hz screen refresh, decoupled from the control loop above
    lastDraw = millis();

    oledHeader("LINE FOLLOWING");
    display.setCursor(0, 14);
    if (navMode == NAV_TRACK)       { display.print("Err: "); display.println((int)error); }
    else if (navMode == NAV_SEARCH) { display.print("Search: "); display.println(lastEdgeDir < 0 ? "<-" : lastEdgeDir > 0 ? "->" : "?"); }
    else                             { display.println("STOPPED - lost"); }
    display.setCursor(0, 24);
    display.print("L:"); display.print(left);
    display.print("  R:"); display.print(right);
    display.setCursor(90, 24);
    display.print(directionArrow(left, right));
    display.setCursor(0, 40);
    display.println(lineFound ? "Line: OK" : (navMode == NAV_TRACK ? "Line: LOST" : "Line: SEARCH"));
    oledFooter("BACK = Stop");
    display.display();

#if DEBUG_SERIAL
    Serial.print("nav="); Serial.print((int)navMode);
    Serial.print(" err="); Serial.print(error);
    Serial.print(" L="); Serial.print(left);
    Serial.print(" R="); Serial.print(right);
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
int ledChannel = 0; // 0=R,1=G,2=B
const char* ledChannelName[3] = {"R", "G", "B"};

void ledColorLoop() {
  oledHeader("LED COLOR");
  display.setCursor(0, 14);
  display.print("Channel: "); display.println(ledChannelName[ledChannel]);
  display.setCursor(0, 24);
  display.print("R:"); display.print(idleR);
  display.print(" G:"); display.print(idleG);
  display.print(" B:"); display.println(idleB);
  oledFooter("UP=- DN=+ OK=Next");
  display.setCursor(0, 46);
  display.println("BACK=Save & Back");
  display.display();

  uint8_t* chan = (ledChannel == 0) ? &idleR : (ledChannel == 1) ? &idleG : &idleB;

  if (up())   { *chan = (uint8_t)max(0, *chan - 10); beep(1600,20); }
  if (down()) { *chan = (uint8_t)min(255, *chan + 10); beep(2000,20); }
  if (select()) { ledChannel = (ledChannel + 1) % 3; beep(2200,20); }

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
int pidParamIndex = 0; // 0=Kp,1=Ki,2=Kd,3=BaseSpeed,4=TestSpeed
const char* settingsParamName[5] = {"Kp", "Ki", "Kd", "BaseSpd", "TestSpd"};
const int settingsParamCount = 5;

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
  }
  display.setTextSize(1);

  oledFooter("UP=Param DN/OK=-/+ BK=Sav");
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
  }
  delay(5); // small yield
}
