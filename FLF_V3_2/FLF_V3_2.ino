/*
  =====================================================================
  COMPETITION-PROOF LINE FOLLOWER ROBOT — V3.2 (ESP32-S3-Pico)
  =====================================================================
  Target Hardware:
    - MCU: Waveshare ESP32-S3-Pico (ESP32-S3 Dev Module)
    - Sensors: 14x TCRT5000 IR reflectance array via CD74HC4067 multiplexer
    - Motors: 2x N30 Micro Gearmotors with quadrature encoders
    - Motor Drivers: Dual BTS7960 H-Bridge (PWM forward / reverse)
    - Display: 0.96" I2C OLED SSD1306 (128x64)
    - Buttons: 4x Tactile Push Buttons (Debounced, logical role mapping)
    - Status LEDs: 2x WS2812B Addressable RGB LEDs
    - Buzzer: Passive piezo on LEDC tone channel

  Advanced Competition Features in V3.2:
    1. Multi-Stage Anti-Loss Line Recovery:
       - Wing sensor memory (Sensors 0..2 on Left, 11..13 on Right) records
         exact timestamp and encoder ticks of outer triggers.
       - Immediate edge-guided reverse-pivot recovery sweeps back directly
         onto line upon leaving sharp 90-degree corners.
       - Encoder-tick & time-bounded search pattern prevents spinning in circles.
    2. Dashed Line / Gap Bridging (Forward Tangent Glide):
       - Momentary white space on straightaways / gentle curves keeps
         driving forward along the line's last trajectory without panic spinning.
    3. T-Junction, Crossroad & Priority Turning:
       - Real-time intersection classifier.
       - User-selectable junction priority: LEFT, RIGHT, or STRAIGHT (tuned
         via OLED menu and saved to NVS flash).
       - Smart blind-traversal through intersection crossbars.
    4. Dynamic Throttle & Traction Control:
       - Straightaway acceleration boost up to maxSpeed when well-centered.
       - Automatic corner braking / speed reduction during high error / hairpins.
       - Slew-rate filtering prevents wheel slip and gearbox shock.
    5. Dual-Mode Calibration & Diagnostics:
       - Manual hand-sweep or one-touch Auto 360-degree spin calibration.
       - Real-time contrast quality index (OK / LOW) and active sensor tick bar.
       - Polarity invert toggle (Dark on Light vs Light on Dark).
    6. Readback-Verified NVS Flash Persistence:
       - Calibration, PID constants, speeds, priority mode, and LED colors.
  =====================================================================
*/

#include <Wire.h>
#include <math.h>
#include <string.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// =====================================================================
//  TYPE DEFINITIONS
// =====================================================================
enum State {
  STATE_MENU,
  STATE_CALIBRATION,
  STATE_MOTOR_TEST,
  STATE_LINE_FOLLOW,
  STATE_LED_COLOR,
  STATE_SETTINGS
};

enum JunctionPriority {
  PRIORITY_LEFT = 0,
  PRIORITY_RIGHT = 1,
  PRIORITY_STRAIGHT = 2
};

enum NavMode {
  NAV_TRACK,        // Normal PID line following
  NAV_GAP_BRIDGING, // Dashed line / white gap momentum traversal
  NAV_JUNCTION,     // Handling T-junction, crossroad, or 90 deg turn
  NAV_SEARCH,       // Edge-guided active line recovery
  NAV_GIVEUP        // Safety timeout stop with alarm
};

enum CalibMode { CALIB_MANUAL, CALIB_AUTO };
enum CalibPhase { CALIB_SELECT_MODE, CALIB_RUNNING, CALIB_AUTO_DONE };
enum MotorTestTarget { MT_LEFT, MT_RIGHT, MT_BOTH };

struct RGB { uint8_t r, g, b; };

struct Button {
  uint8_t pin;
  bool lastReading = HIGH;
  bool stableState = HIGH;
  unsigned long lastChange = 0;
};

// =====================================================================
//  PIN DEFINITIONS (Waveshare ESP32-S3-Pico Wiring)
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

// ---- Buttons ----
#define SW1       10   // Logical DOWN
#define SW2       11   // Logical BACK
#define SW3       12   // Logical SELECT / OK
#define SW4       13   // Logical UP

// ---- Right Motor Driver (BTS7960) ----
#define R_RPWM    15
#define R_LPWM    16

// ---- Left Motor Driver (BTS7960) ----
#define L_LPWM    33
#define L_RPWM    34

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
//  CONSTANTS & CONFIGURATION
// =====================================================================
#define NUM_SENSORS 14
const uint8_t muxChannelForSensor[NUM_SENSORS] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};

#define PWM_FREQ  20000
#define PWM_RES   8      // 0-255

// Set to actual output shaft counts per rev if known (0 = fallback to counts/sec)
const int ENCODER_COUNTS_PER_REV = 0;

// Enable Serial Telemetry (115200 baud)
#define DEBUG_SERIAL 1

// ---- Sensor Geometry Constants ----
// Sensor positions centered around 0 (Span: -6500 to +6500)
const long SENSOR_POS_MIN = -6500L;
const long SENSOR_POS_MAX = 6500L;
const int ON_THRESH = 450;             // lineWeight() > 450 considered active on line

// ---- Default Control Tuning Parameters ----
float Kp = 0.058f;
float Ki = 0.0004f;
float Kd = 0.42f;

int baseSpeed = 150;                   // Base cruising speed (0-255)
int maxSpeed  = 210;                   // Straightaway boost speed (0-255)
int testSpeed = 175;                   // Manual motor jog speed
int navTurnSpeed = 120;                // Dedicated pivot speed for recovery

// Junction Priority Setting (0=LEFT, 1=RIGHT, 2=STRAIGHT)
JunctionPriority junctionPriority = PRIORITY_LEFT;
const char* junctionPriorityNames[3] = {"LEFT", "RIGHT", "STRAIGHT"};

// ---- Recovery & Gap Timing ----
const unsigned long GAP_GRACE_MS        = 220;   // Forward glide window on dashed line / white gap
const unsigned long EDGE_MEMORY_MS      = 550;   // Time window to trust recent outer wing trigger
const unsigned long SEARCH_TIME_MAX_MS  = 2000;  // Max duration for recovery sweep before safety stop
const long SEARCH_MAX_TICKS             = 160;   // Encoder tick limit for recovery search
const unsigned long JUNCTION_BLIND_MS   = 160;   // Blind traversal window through intersection bar

// =====================================================================
//  GLOBAL OBJECTS
// =====================================================================
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences prefs;

// =====================================================================
//  STATE MACHINE GLOBALS
// =====================================================================
State currentState = STATE_MENU;

bool calibResetFlag = false;
bool lineFollowStarting = false;

CalibMode calibMode = CALIB_MANUAL;
CalibPhase calibPhase = CALIB_SELECT_MODE;
unsigned long calibAutoStart = 0;
const unsigned long CALIB_AUTO_DURATION_MS = 5500;
const int CALIB_AUTO_SPEED = 140;

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
//  SENSOR DATA & NORMALIZATION
// =====================================================================
int sensorRaw[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorNorm[NUM_SENSORS]; // 0-1000 normalized

bool lineIsDark = true;      // True = dark line on light surface; False = inverted

// =====================================================================
//  ENCODER TRACKING & RATES
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

long lastLeftEncForRate = 0, lastRightEncForRate = 0;
unsigned long lastRateCalc = 0;
float leftCPS = 0, rightCPS = 0;
float leftRPM = 0, rightRPM = 0;

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
//  LED STATUS SYSTEM
// =====================================================================
uint8_t idleR = 0, idleG = 0, idleB = 70;

RGB colorForState(State s) {
  switch (s) {
    case STATE_CALIBRATION: return {255, 190, 0};   // amber/yellow
    case STATE_MOTOR_TEST:  return {170, 0, 255};   // purple
    case STATE_LINE_FOLLOW: return {0, 255, 0};     // green
    case STATE_LED_COLOR:   return {idleR, idleG, idleB};
    case STATE_SETTINGS:    return {0, 200, 255};   // cyan
    default:                return {idleR, idleG, idleB};
  }
}

void setLED(RGB c) {
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(c.r, c.g, c.b));
  strip.show();
}

void applyModeLED() { setLED(colorForState(currentState)); }

void updateIdleBreath() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 30) return;
  lastUpdate = millis();
  float phase = (millis() % 3000) / 3000.0f * TWO_PI;
  float b = 0.25f + (sinf(phase) * 0.5f + 0.5f) * 0.75f;
  setLED({(uint8_t)(idleR * b), (uint8_t)(idleG * b), (uint8_t)(idleB * b)});
}

void updateLineFollowLED(NavMode mode, bool lineFound) {
  static unsigned long lastFlash = 0;
  static bool flashState = false;

  switch (mode) {
    case NAV_TRACK:
      setLED({0, 255, 0}); // Solid Green on track
      break;
    case NAV_GAP_BRIDGING:
      setLED({0, 180, 255}); // Solid Cyan while crossing dashed line
      break;
    case NAV_JUNCTION:
      setLED({255, 160, 0}); // Orange at junction
      break;
    case NAV_SEARCH:
      if (millis() - lastFlash > 120) { flashState = !flashState; lastFlash = millis(); }
      setLED(flashState ? RGB{255, 0, 255} : RGB{0, 0, 0}); // Fast Magenta Flash
      break;
    case NAV_GIVEUP:
      if (millis() - lastFlash > 150) { flashState = !flashState; lastFlash = millis(); }
      setLED(flashState ? RGB{255, 0, 0} : RGB{0, 0, 0}); // Red strobe on lost
      break;
  }
}

// =====================================================================
//  NON-BLOCKING BUZZER
// =====================================================================
unsigned long buzzerStopAt = 0;

void beep(int freq = 2200, int dur = 60) {
  ledcWriteTone(BUZZER_PIN, freq);
  buzzerStopAt = millis() + dur;
}

void buzzerService() {
  if (buzzerStopAt && (long)(millis() - buzzerStopAt) >= 0) {
    ledcWriteTone(BUZZER_PIN, 0);
    buzzerStopAt = 0;
  }
}

void beepOK()        { beep(2600, 70); }
void beepBack()      { beep(850, 100); }
void beepError()     { beep(450, 280); }
void beepJunction()  { beep(2900, 40); }
void beepStart()     { beep(1600, 90); delay(100); beep(2700, 140); }

// =====================================================================
//  BUTTON HANDLING (Debounced & Logical Mapping)
// =====================================================================
Button btn1{SW1}, btn2{SW2}, btn3{SW3}, btn4{SW4};
const unsigned long DEBOUNCE_MS = 30;

bool wasPressed(Button &b) {
  bool reading = digitalRead(b.pin);
  bool pressedEdge = false;
  if (reading != b.lastReading) b.lastChange = millis();
  if ((millis() - b.lastChange) > DEBOUNCE_MS) {
    if (reading != b.stableState) {
      b.stableState = reading;
      if (b.stableState == LOW) pressedEdge = true;
    }
  }
  b.lastReading = reading;
  return pressedEdge;
}

bool up()     { return wasPressed(btn4); }  // Physical SW4
bool down()   { return wasPressed(btn1); }  // Physical SW1
bool select() { return wasPressed(btn3); }  // Physical SW3
bool back()   { return wasPressed(btn2); }  // Physical SW2

bool upHeld()   { return digitalRead(SW4) == LOW; }
bool downHeld() { return digitalRead(SW1) == LOW; }

// =====================================================================
//  MULTIPLEXER & SENSOR ACQUISITION
// =====================================================================
void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
  delayMicroseconds(6); // Fast analog settling
}

void readAllSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(muxChannelForSensor[i]);
    sensorRaw[i] = analogRead(MUX_SIG); // 0-4095
  }
}

void normalizeSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    int lo = sensorMin[i], hi = sensorMax[i];
    if (hi <= lo + 50) { sensorNorm[i] = 0; continue; }
    int v = constrain(sensorRaw[i], lo, hi);
    sensorNorm[i] = map(v, lo, hi, 0, 1000);
  }
}

int lineWeight(int i) {
  return lineIsDark ? sensorNorm[i] : (1000 - sensorNorm[i]);
}

long sensorPosition(int i) {
  return (long)i * 1000L - 6500L;
}

// =====================================================================
//  MOTOR CONTROL & SLEW-RATE LIMITING
// =====================================================================
int currentLeftPWM = 0;
int currentRightPWM = 0;
const int PWM_SLEW_LIMIT = 45; // Max PWM change per loop cycle to preserve traction

void motorsInit() {
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
}

void setRightMotorRaw(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(R_RPWM, speed); ledcWrite(R_LPWM, 0); }
  else            { ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, -speed); }
}

void setLeftMotorRaw(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(L_RPWM, speed); ledcWrite(L_LPWM, 0); }
  else            { ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, -speed); }
}

void setMotorsSmooth(int targetL, int targetR, bool immediate = false) {
  targetL = constrain(targetL, -255, 255);
  targetR = constrain(targetR, -255, 255);

  if (immediate) {
    currentLeftPWM = targetL;
    currentRightPWM = targetR;
  } else {
    currentLeftPWM  += constrain(targetL - currentLeftPWM, -PWM_SLEW_LIMIT, PWM_SLEW_LIMIT);
    currentRightPWM += constrain(targetR - currentRightPWM, -PWM_SLEW_LIMIT, PWM_SLEW_LIMIT);
  }

  setLeftMotorRaw(currentLeftPWM);
  setRightMotorRaw(currentRightPWM);
}

void stopMotors() {
  currentLeftPWM = 0;
  currentRightPWM = 0;
  setLeftMotorRaw(0);
  setRightMotorRaw(0);
}

// =====================================================================
//  PREFERENCES (Flash Storage)
// =====================================================================
void confirmSaveResult(bool ok) {
  RGB restore = colorForState(currentState);
  if (ok) {
    beep(2400, 40); delay(70); beep(2800, 50); delay(70);
    setLED({0, 255, 0}); delay(150);
  } else {
    beep(350, 300); delay(320);
    setLED({255, 0, 0}); delay(300);
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

  bool ok = (r1 == sizeof(chkMin)) && (r2 == sizeof(chkMax)) &&
            (memcmp(chkMin, sensorMin, sizeof(chkMin)) == 0) &&
            (memcmp(chkMax, sensorMax, sizeof(chkMax)) == 0) && invertOk;
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
  bool ok = (prefs.getUChar("lr", 0) == idleR) &&
            (prefs.getUChar("lg", 0) == idleG) &&
            (prefs.getUChar("lb", 0) == idleB);
  prefs.end();
  confirmSaveResult(ok);
  return ok;
}

void loadLedColor() {
  prefs.begin("linefw", true);
  idleR = prefs.getUChar("lr", 0);
  idleG = prefs.getUChar("lg", 0);
  idleB = prefs.getUChar("lb", 70);
  prefs.end();
}

bool saveSettings() {
  prefs.begin("linefw", false);
  prefs.putFloat("kp", Kp);
  prefs.putFloat("ki", Ki);
  prefs.putFloat("kd", Kd);
  prefs.putInt("base", baseSpeed);
  prefs.putInt("maxs", maxSpeed);
  prefs.putInt("tspd", testSpeed);
  prefs.putInt("prio", (int)junctionPriority);
  prefs.end();

  prefs.begin("linefw", true);
  bool ok = (prefs.getFloat("kp", Kp + 1) == Kp) &&
            (prefs.getFloat("ki", Ki + 1) == Ki) &&
            (prefs.getFloat("kd", Kd + 1) == Kd) &&
            (prefs.getInt("base", baseSpeed + 1) == baseSpeed) &&
            (prefs.getInt("maxs", maxSpeed + 1) == maxSpeed) &&
            (prefs.getInt("tspd", testSpeed + 1) == testSpeed) &&
            (prefs.getInt("prio", -1) == (int)junctionPriority);
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
  maxSpeed = prefs.getInt("maxs", maxSpeed);
  testSpeed = prefs.getInt("tspd", testSpeed);
  junctionPriority = (JunctionPriority)prefs.getInt("prio", (int)PRIORITY_LEFT);
  prefs.end();
}

// =====================================================================
//  OLED DISPLAY HELPERS
// =====================================================================
void oledHeader(const char* title) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);
}

void oledFooter(const char* hint) {
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print(hint);
  display.setTextColor(SSD1306_WHITE);
}

// =====================================================================
//  MODE: MAIN MENU
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
      case 0: enterState(STATE_CALIBRATION); calibPhase = CALIB_SELECT_MODE; break;
      case 1: enterState(STATE_MOTOR_TEST); break;
      case 2: enterState(STATE_LINE_FOLLOW); lineFollowStarting = true; break;
      case 3: enterState(STATE_LED_COLOR); break;
      case 4: enterState(STATE_SETTINGS); break;
    }
  }
}

// =====================================================================
//  MODE: CALIBRATION
// =====================================================================
void calibrationLoop() {
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

  bool spinning = (calibMode == CALIB_AUTO && calibPhase == CALIB_RUNNING);
  if (spinning) {
    if (millis() - calibAutoStart < CALIB_AUTO_DURATION_MS) {
      setLeftMotorRaw(CALIB_AUTO_SPEED);
      setRightMotorRaw(-CALIB_AUTO_SPEED);
    } else {
      stopMotors();
      calibPhase = CALIB_AUTO_DONE;
      spinning = false;
      beepStart();
    }
  }

  readAllSensors();
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
    if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
  }

  if (up()) { lineIsDark = !lineIsDark; beep(1700, 20); }
  normalizeSensors();

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 40) {
    lastDraw = millis();
    long totalSpread = 0;
    for (int i = 0; i < NUM_SENSORS; i++) totalSpread += (sensorMax[i] - sensorMin[i]);
    bool goodContrast = (totalSpread / NUM_SENSORS) > 800;

    oledHeader(spinning ? "CALIB (spinning)" : "CALIB");
    display.setCursor(80, 0);
    display.print(lineIsDark ? "DRK" : "LIT");
    display.setCursor(108, 0);
    display.print(goodContrast ? "OK" : "LOW");

    const int tickY      = 11;
    const int barTop     = 13;
    const int barBottom  = OLED_H;
    const int barMaxH    = barBottom - barTop;
    int slot = OLED_W / NUM_SENSORS;
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
      display.drawRect(x, barTop, w, barMaxH, SSD1306_WHITE);
      display.fillRect(x + 1, barBottom - h, max(w - 2, 1), h, SSD1306_WHITE);
      if (lineWeight(i) > ON_THRESH) display.fillRect(x, tickY, w, 1, SSD1306_WHITE);
    }

    if (spinning) oledFooter("Spinning.. BK=Stop");
    else oledFooter("OK=Save BK=Cancel UP=Inv");
    display.display();
  }

  if (select()) {
    stopMotors();
    saveCalibration();
    calibPhase = CALIB_SELECT_MODE;
    enterState(STATE_MENU);
  }
  if (back()) {
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
MotorTestTarget motorTestTarget = MT_BOTH;

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
  if (driving && !wasDriving) beep(dir > 0 ? 1400 : 700, 40);
  wasDriving = driving;

  int cmd = dir * testSpeed;
  switch (motorTestTarget) {
    case MT_LEFT:  setLeftMotorRaw(cmd); setRightMotorRaw(0);   break;
    case MT_RIGHT: setLeftMotorRaw(0);   setRightMotorRaw(cmd); break;
    case MT_BOTH:  setLeftMotorRaw(cmd); setRightMotorRaw(cmd); break;
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 50) {
    lastDraw = millis();
    oledHeader("MOTOR TEST");
    display.setCursor(0, 12);
    display.print(motorTestTarget == MT_LEFT  ? "[L] " : " L  ");
    display.print(motorTestTarget == MT_RIGHT ? "[R] " : " R  ");
    display.print(motorTestTarget == MT_BOTH  ? "[BOTH]" : " BOTH ");
    display.setCursor(0, 22);
    display.println(dir > 0 ? "-> FWD" : dir < 0 ? "<- REV" : "-- STOP");

    display.setCursor(0, 34);
    display.print("L: "); display.print(leftEncCount); display.print(" ");
    if (ENCODER_COUNTS_PER_REV > 0) { display.print(leftRPM, 0); display.println("rpm"); }
    else { display.print(leftCPS, 0); display.println("cps"); }

    display.setCursor(0, 44);
    display.print("R: "); display.print(rightEncCount); display.print(" ");
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
//  MODE: COMPETITION LINE FOLLOWING ENGINE
// =====================================================================
float pidIntegral = 0;
float pidLastError = 0;
float lastValidError = 0;
unsigned long lastLineLostBeep = 0;
unsigned long lineLostStartTime = 0;
unsigned long onLineStartTime = 0;
int currentThrottle = 150;

// Wing Memory
unsigned long lastLeftWingTime = 0;
unsigned long lastRightWingTime = 0;
int lastKnownTurnBias = 0; // -1=Left, 1=Right

// Navigation State
NavMode navMode = NAV_TRACK;
unsigned long navPhaseStartTime = 0;
long navEncStartTicks = 0;
unsigned long junctionDetectTime = 0;

const char* directionArrow(int left, int right) {
  if (left > right + 15) return "<<<";
  if (right > left + 15) return ">>>";
  if (left > 0)          return "^^^";
  if (left < 0)          return "vvv";
  return "---";
}

void lineFollowLoop() {
  if (lineFollowStarting) {
    pidIntegral = 0;
    pidLastError = 0;
    lastValidError = 0;
    lastLeftWingTime = 0;
    lastRightWingTime = 0;
    lastKnownTurnBias = 0;
    navMode = NAV_TRACK;
    currentThrottle = baseSpeed;
    onLineStartTime = millis();
    lineLostStartTime = 0;
    beepStart();
    lineFollowStarting = false;
  }

  readAllSensors();
  normalizeSensors();

  long weightedSum = 0;
  long sum = 0;
  int activeSensorsCount = 0;
  bool leftWingActive = false;
  bool rightWingActive = false;
  bool centerActive = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int w = lineWeight(i);
    weightedSum += (long)w * sensorPosition(i);
    sum += w;

    if (w > ON_THRESH) {
      activeSensorsCount++;
      if (i <= 2) leftWingActive = true;
      if (i >= 11) rightWingActive = true;
      if (i >= 5 && i <= 8) centerActive = true;
    }
  }

  unsigned long now = millis();
  long encAvg = (leftEncCount + rightEncCount) / 2;

  // Track Wing Triggers for recovery memory
  if (leftWingActive)  { lastLeftWingTime = now; lastKnownTurnBias = -1; }
  if (rightWingActive) { lastRightWingTime = now; lastKnownTurnBias = 1; }

  bool lineFound = (sum > (long)(NUM_SENSORS * 35)) && (activeSensorsCount > 0);

  // Check for Junctions (Cross, T-Junction, Wide Bar: >= 7 sensors or Left+Right+Center active)
  bool isJunction = (activeSensorsCount >= 7) || (leftWingActive && rightWingActive && centerActive);

  // Error Calculation
  float error = 0;
  if (lineFound) {
    error = (float)weightedSum / (float)sum;
    lastValidError = error;
    lineLostStartTime = 0;
  } else {
    if (lineLostStartTime == 0) lineLostStartTime = now;
  }

  int targetLeft = 0;
  int targetRight = 0;

  // -------------------------------------------------------------
  // NAVIGATION STATE MACHINE
  // -------------------------------------------------------------
  if (isJunction && navMode != NAV_GIVEUP) {
    if (navMode != NAV_JUNCTION) {
      navMode = NAV_JUNCTION;
      navPhaseStartTime = now;
      navEncStartTicks = encAvg;
      junctionDetectTime = now;
      beepJunction();
    }
  }

  switch (navMode) {
    // -----------------------------------------------------------
    // 1. NORMAL PID LINE TRACKING
    // -----------------------------------------------------------
    case NAV_TRACK: {
      if (!lineFound) {
        // Line lost! Determine whether to bridge a gap or initiate recovery
        if (fabs(lastValidError) < 2200.0f && (now - lineLostStartTime) < GAP_GRACE_MS) {
          navMode = NAV_GAP_BRIDGING;
          navPhaseStartTime = now;
        } else {
          navMode = NAV_SEARCH;
          navPhaseStartTime = now;
          navEncStartTicks = encAvg;
          beep(1400, 40);
        }
        break;
      }

      // Dynamic Throttle / Straightaway Boost
      if (fabs(error) < 600.0f) {
        if (currentThrottle < maxSpeed) currentThrottle += 2;
      } else if (fabs(error) > 2800.0f) {
        // Corner Braking: reduce cruise speed into sharp hairpins for maximum grip
        currentThrottle = max(baseSpeed - 20, (int)(baseSpeed * 0.85f));
      } else {
        if (currentThrottle > baseSpeed) currentThrottle -= 3;
      }

      // PID Math
      pidIntegral += error;
      pidIntegral = constrain(pidIntegral, -40000, 40000);
      float derivative = error - pidLastError;
      float output = Kp * error + Ki * pidIntegral + Kd * derivative;
      pidLastError = error;

      targetLeft  = currentThrottle - (int)output;
      targetRight = currentThrottle + (int)output;
      setMotorsSmooth(targetLeft, targetRight);
      break;
    }

    // -----------------------------------------------------------
    // 2. DASHED LINE / WHITE GAP MOMENTUM BRIDGING
    // -----------------------------------------------------------
    case NAV_GAP_BRIDGING: {
      if (lineFound) {
        navMode = NAV_TRACK;
        pidLastError = error;
        break;
      }

      unsigned long timeInGap = now - navPhaseStartTime;
      if (timeInGap > GAP_GRACE_MS) {
        // Gap ended without finding line -> Transition to active recovery
        navMode = NAV_SEARCH;
        navPhaseStartTime = now;
        navEncStartTicks = encAvg;
        beep(1400, 40);
        break;
      }

      // Glide straight along last vector with mild damping
      int glideSpeed = max(baseSpeed - 15, 110);
      float mildCorrection = lastValidError * (Kp * 0.4f);
      targetLeft  = glideSpeed - (int)mildCorrection;
      targetRight = glideSpeed + (int)mildCorrection;
      setMotorsSmooth(targetLeft, targetRight);
      break;
    }

    // -----------------------------------------------------------
    // 3. T-JUNCTION & CROSSROAD PRIORITY EXECUTION
    // -----------------------------------------------------------
    case NAV_JUNCTION: {
      unsigned long timeInJunction = now - navPhaseStartTime;

      if (junctionPriority == PRIORITY_LEFT) {
        // Pivot Left
        targetLeft = -navTurnSpeed;
        targetRight = navTurnSpeed;
        if (timeInJunction > JUNCTION_BLIND_MS && lineFound && fabs(error) < 2500) {
          navMode = NAV_TRACK;
        }
      } else if (junctionPriority == PRIORITY_RIGHT) {
        // Pivot Right
        targetLeft = navTurnSpeed;
        targetRight = -navTurnSpeed;
        if (timeInJunction > JUNCTION_BLIND_MS && lineFound && fabs(error) < 2500) {
          navMode = NAV_TRACK;
        }
      } else { // PRIORITY_STRAIGHT
        // Drive straight forward across the junction bar
        targetLeft = baseSpeed;
        targetRight = baseSpeed;
        if (timeInJunction > JUNCTION_BLIND_MS && lineFound && activeSensorsCount <= 4) {
          navMode = NAV_TRACK;
        }
      }

      // Junction safety fallback
      if (timeInJunction > 900) navMode = NAV_TRACK;
      setMotorsSmooth(targetLeft, targetRight);
      break;
    }

    // -----------------------------------------------------------
    // 4. MULTI-TIERED ACTIVE LINE RECOVERY (SEARCH)
    // -----------------------------------------------------------
    case NAV_SEARCH: {
      if (lineFound) {
        navMode = NAV_TRACK;
        pidLastError = error;
        lineLostStartTime = 0;
        beepOK();
        break;
      }

      long searchTicks = labs(encAvg - navEncStartTicks);
      unsigned long searchTime = now - navPhaseStartTime;

      if (searchTicks > SEARCH_MAX_TICKS || searchTime > SEARCH_TIME_MAX_MS) {
        navMode = NAV_GIVEUP;
        stopMotors();
        beepError();
        break;
      }

      bool recentLeft  = (lastLeftWingTime > 0) && (now - lastLeftWingTime < EDGE_MEMORY_MS);
      bool recentRight = (lastRightWingTime > 0) && (now - lastRightWingTime < EDGE_MEMORY_MS);

      if (recentLeft && !recentRight) {
        // Line slipped off Left wing -> Sharp reverse-pivot Left
        targetLeft = -navTurnSpeed;
        targetRight = navTurnSpeed;
      } else if (recentRight && !recentLeft) {
        // Line slipped off Right wing -> Sharp reverse-pivot Right
        targetLeft = navTurnSpeed;
        targetRight = -navTurnSpeed;
      } else if (lastKnownTurnBias == -1) {
        targetLeft = (searchTime < 450) ? -navTurnSpeed : (navTurnSpeed / 3);
        targetRight = navTurnSpeed;
      } else if (lastKnownTurnBias == 1) {
        targetLeft = navTurnSpeed;
        targetRight = (searchTime < 450) ? -navTurnSpeed : (navTurnSpeed / 3);
      } else {
        // Symmetrical sweep
        if (searchTime < 300) {
          targetLeft = -navTurnSpeed; targetRight = navTurnSpeed;
        } else if (searchTime < 650) {
          targetLeft = navTurnSpeed; targetRight = -navTurnSpeed;
        } else {
          targetLeft = navTurnSpeed; targetRight = navTurnSpeed / 2;
        }
      }

      setMotorsSmooth(targetLeft, targetRight);
      break;
    }

    // -----------------------------------------------------------
    // 5. SAFETY TIMEOUT / GIVEUP
    // -----------------------------------------------------------
    case NAV_GIVEUP: {
      stopMotors();
      if (lineFound) {
        // User placed robot back on the line -> Auto resume
        navMode = NAV_TRACK;
        pidLastError = error;
        beepStart();
      }
      break;
    }
  }

  // Periodic audible warning while line is lost
  if (!lineFound && navMode != NAV_TRACK && now - lastLineLostBeep > 500) {
    beep(600, 100);
    lastLineLostBeep = now;
  }

  updateLineFollowLED(navMode, lineFound);

  // OLED & Serial Telemetry (~20 Hz)
  static unsigned long lastDraw = 0;
  if (now - lastDraw >= 50) {
    lastDraw = now;

    oledHeader("LINE FOLLOWING");
    display.setCursor(0, 14);
    if (navMode == NAV_TRACK) {
      display.print("Err:"); display.print((int)error);
      display.setCursor(70, 14);
      display.print("Spd:"); display.println(currentThrottle);
    } else if (navMode == NAV_GAP_BRIDGING) {
      display.println("MODE: GAP BRIDGE");
    } else if (navMode == NAV_JUNCTION) {
      display.print("JUNC: "); display.println(junctionPriorityNames[junctionPriority]);
    } else if (navMode == NAV_SEARCH) {
      display.print("SEARCH: "); display.println(lastKnownTurnBias < 0 ? "LEFT" : "RIGHT");
    } else {
      display.println("STOPPED - LINE LOST");
    }

    display.setCursor(0, 26);
    display.print("L:"); display.print(currentLeftPWM);
    display.print(" R:"); display.print(currentRightPWM);
    display.setCursor(95, 26);
    display.print(directionArrow(currentLeftPWM, currentRightPWM));

    display.setCursor(0, 38);
    display.print("Sensors: "); display.print(activeSensorsCount);
    display.print("/14  ");
    display.println(lineFound ? "OK" : "LOST");

    oledFooter("BACK = Stop");
    display.display();

#if DEBUG_SERIAL
    Serial.print("mode="); Serial.print((int)navMode);
    Serial.print(" err="); Serial.print(error);
    Serial.print(" thr="); Serial.print(currentThrottle);
    Serial.print(" L="); Serial.print(currentLeftPWM);
    Serial.print(" R="); Serial.print(currentRightPWM);
    Serial.print(" act="); Serial.println(activeSensorsCount);
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
int ledChannel = 0;
const char* ledChannelName[4] = {"R", "G", "B", "PRESET"};

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

  setLED({idleR, idleG, idleB});

  if (back()) {
    saveLedColor();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  MODE: SETTINGS (PID, Speeds & Junction Priority)
// =====================================================================
int settingsIndex = 0;
const char* settingsNames[7] = {"Kp", "Ki", "Kd", "BaseSpd", "MaxBoost", "TestSpd", "Priority"};
const int settingsCount = 7;

void settingsLoop() {
  oledHeader("SETTINGS");
  display.setCursor(0, 14);
  display.print("Param: "); display.println(settingsNames[settingsIndex]);

  display.setTextSize(2);
  display.setCursor(0, 28);
  switch (settingsIndex) {
    case 0: display.println(Kp, 3); break;
    case 1: display.println(Ki, 4); break;
    case 2: display.println(Kd, 2); break;
    case 3: display.println(baseSpeed); break;
    case 4: display.println(maxSpeed); break;
    case 5: display.println(testSpeed); break;
    case 6: display.println(junctionPriorityNames[junctionPriority]); break;
  }
  display.setTextSize(1);

  oledFooter("UP=Param DN/OK=-/+ BK=Sav");
  display.display();

  if (up()) { settingsIndex = (settingsIndex + 1) % settingsCount; beep(1800, 20); }

  bool dec = down();
  bool inc = select();
  if (dec || inc) {
    int dir = inc ? 1 : -1;
    switch (settingsIndex) {
      case 0: Kp = max(0.0f, Kp + dir * 0.005f); break;
      case 1: Ki = max(0.0f, Ki + dir * 0.0001f); break;
      case 2: Kd = max(0.0f, Kd + dir * 0.02f); break;
      case 3: baseSpeed = constrain(baseSpeed + dir * 10, 50, 255); break;
      case 4: maxSpeed  = constrain(maxSpeed + dir * 10, baseSpeed, 255); break;
      case 5: testSpeed = constrain(testSpeed + dir * 10, 50, 255); break;
      case 6:
        junctionPriority = (JunctionPriority)((junctionPriority + dir + 3) % 3);
        break;
    }
    beep(2000, 20);
  }

  if (back()) {
    saveSettings();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

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

  // Multiplexer
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  analogReadResolution(12);

  // Buttons
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
  Wire.setClock(400000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    for (int i = 0; i < 3; i++) {
      setLED({255, 0, 0}); delay(150);
      setLED({0, 0, 0}); delay(150);
    }
  }
  display.clearDisplay();
  display.display();

  // Load Saved Preferences
  loadCalibration();
  loadLedColor();
  loadSettings();

  applyModeLED();
  beepOK();
}

// =====================================================================
//  MAIN LOOP
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
    case STATE_SETTINGS:    settingsLoop();    break;
  }
  delay(2);
}
