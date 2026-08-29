            /*
  =====================================================================
  LINE FOLLOWER ROBOT — ESP32-S3 Pico
  =====================================================================
  Features:
    - OLED + 4-button menu UI
    - Sensor calibration (14x TCRT5000 via CD74HC4067 mux)
    - Motor test mode (2x N30 motors w/ encoder, BTS7960 drivers)
    - PID line following
    - WS2812 RGB LED (x2) that changes color per mode, plus a
      manually-adjustable "idle" color from the menu
    - Passive buzzer feedback beeps
    - Calibration + LED color saved to flash (Preferences/NVS)

  Required libraries (Install via Library Manager):
    - Adafruit GFX Library
    - Adafruit SSD1306
    - Adafruit NeoPixel
    (Preferences.h is built into the ESP32 Arduino core)

  Board: "ESP32S3 Dev Module" (or Waveshare ESP32-S3-Pico variant)
  Arduino-ESP32 core: 3.x (uses ledcAttach/ledcWrite-by-pin API and tone())
  =====================================================================
*/

#include <Wire.h>
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

// ---- Buttons ----
#define SW1       10   // UP
#define SW2       11   // DOWN
#define SW3       12   // SELECT / OK
#define SW4       13   // BACK / STOP

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
#define BUZZER_PIN 38

// =====================================================================
//  CONSTANTS
// =====================================================================
#define NUM_SENSORS 14
const uint8_t muxChannelForSensor[NUM_SENSORS] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};

#define PWM_FREQ  20000
#define PWM_RES   8      // 0-255

// PID defaults (tune from the "PID Tune" submenu, or edit here)
float Kp = 0.06;
float Ki = 0.0006;
float Kd = 0.4;
int baseSpeed = 150;     // 0-255, forward cruise speed while line following

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
  "PID Tune"
};
const int menuCount = 5;
int menuIndex = 0;

// =====================================================================
//  SENSOR DATA
// =====================================================================
int sensorRaw[NUM_SENSORS];
int sensorMin[NUM_SENSORS];
int sensorMax[NUM_SENSORS];
int sensorNorm[NUM_SENSORS]; // 0-1000 normalized

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

// =====================================================================
//  BUZZER
// =====================================================================
void beep(int freq = 2000, int dur = 60) {
  tone(BUZZER_PIN, freq, dur);
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

bool up()     { return wasPressed(btn1); }
bool down()   { return wasPressed(btn2); }
bool select() { return wasPressed(btn3); }
bool back()   { return wasPressed(btn4); }

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
//  PREFERENCES (persist calibration + LED color)
// =====================================================================
void saveCalibration() {
  prefs.begin("linefw", false);
  prefs.putBytes("smin", sensorMin, sizeof(sensorMin));
  prefs.putBytes("smax", sensorMax, sizeof(sensorMax));
  prefs.end();
}
void loadCalibration() {
  prefs.begin("linefw", true);
  size_t r1 = prefs.getBytes("smin", sensorMin, sizeof(sensorMin));
  size_t r2 = prefs.getBytes("smax", sensorMax, sizeof(sensorMax));
  prefs.end();
  if (r1 != sizeof(sensorMin) || r2 != sizeof(sensorMax)) {
    for (int i = 0; i < NUM_SENSORS; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  }
}
void saveLedColor() {
  prefs.begin("linefw", false);
  prefs.putUChar("lr", idleR);
  prefs.putUChar("lg", idleG);
  prefs.putUChar("lb", idleB);
  prefs.end();
}
void loadLedColor() {
  prefs.begin("linefw", true);
  idleR = prefs.getUChar("lr", 0);
  idleG = prefs.getUChar("lg", 0);
  idleB = prefs.getUChar("lb", 60);
  prefs.end();
}
void savePID() {
  prefs.begin("linefw", false);
  prefs.putFloat("kp", Kp);
  prefs.putFloat("ki", Ki);
  prefs.putFloat("kd", Kd);
  prefs.putInt("base", baseSpeed);
  prefs.end();
}
void loadPID() {
  prefs.begin("linefw", true);
  Kp = prefs.getFloat("kp", Kp);
  Ki = prefs.getFloat("ki", Ki);
  Kd = prefs.getFloat("kd", Kd);
  baseSpeed = prefs.getInt("base", baseSpeed);
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
    display.setCursor(2, 14 + i * 10);
    display.print(i == menuIndex ? "> " : "  ");
    display.println(menuItems[i]);
  }
  display.display();
}

void handleMenu() {
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

  readAllSensors();
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorRaw[i] < sensorMin[i]) sensorMin[i] = sensorRaw[i];
    if (sensorRaw[i] > sensorMax[i]) sensorMax[i] = sensorRaw[i];
  }

  oledHeader("CALIBRATING");

  // ---- per-sensor bar visualizer (one bar per sensor, live) ----
  const int barTop    = 12;             // just under the header line
  const int barBottom = OLED_H;         // 64
  const int barMaxH   = barBottom - barTop;
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
  }

  // Instructions drawn with an opaque background so they stay readable
  // even when overlapping a tall bar.
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print("SW3=Save  SW4=Cancel");
  display.setTextColor(SSD1306_WHITE);

  display.display();

  if (select()) { // Save & exit
    saveCalibration();
    beepStart();
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
// SW1/SW2 = left motor reverse/forward while held (simple pulse test)
// SW3 = pulse both motors forward briefly
// SW4 = back to menu
int testSpeed = 180;

void motorTestLoop() {
  oledHeader("MOTOR TEST");
  display.setCursor(0, 14);
  display.print("L enc: "); display.println(leftEncCount);
  display.setCursor(0, 24);
  display.print("R enc: "); display.println(rightEncCount);
  display.setCursor(0, 36);
  display.println("SW1=L  SW2=R  SW3=Both");
  display.setCursor(0, 46);
  display.println("SW4=Back");
  display.display();

  // Hold-to-run: read raw pin state directly for responsive testing
  bool p1 = digitalRead(SW1) == LOW;
  bool p2 = digitalRead(SW2) == LOW;
  bool p3 = digitalRead(SW3) == LOW;

  setLeftMotor(p1 ? testSpeed : 0);
  setRightMotor(p2 ? testSpeed : 0);
  if (p3) { setLeftMotor(testSpeed); setRightMotor(testSpeed); }
  if (!p1 && !p2 && !p3) stopMotors();

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

void lineFollowLoop() {
  if (lineFollowStarting) {
    pidIntegral = 0;
    pidLastError = 0;
    beepStart();
    lineFollowStarting = false;
  }

  readAllSensors();
  normalizeSensors();

  long weightedSum = 0;
  long sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (long)sensorNorm[i] * sensorPosition(i);
    sum += sensorNorm[i];
  }

  bool lineFound = sum > (long)(NUM_SENSORS * 30); // threshold: something detected
  float error;
  if (lineFound) {
    error = (float)weightedSum / (float)sum;
  } else {
    // Line lost: keep turning the direction it was last drifting
    error = (pidLastError >= 0) ? 6500 : -6500;
    if (millis() - lastLineLostBeep > 500) { beepError(); lastLineLostBeep = millis(); }
  }

  pidIntegral += error;
  pidIntegral = constrain(pidIntegral, -50000, 50000);
  float derivative = error - pidLastError;
  float output = Kp * error + Ki * pidIntegral + Kd * derivative;
  pidLastError = error;

  int left  = baseSpeed - (int)output;
  int right = baseSpeed + (int)output;
  setLeftMotor(constrain(left, -255, 255));
  setRightMotor(constrain(right, -255, 255));

  oledHeader("LINE FOLLOWING");
  display.setCursor(0, 14);
  display.print("Err: "); display.println((int)error);
  display.setCursor(0, 24);
  display.print("L:"); display.print(left);
  display.print("  R:"); display.println(right);
  display.setCursor(0, 40);
  display.println(lineFound ? "Line: OK" : "Line: LOST");
  display.setCursor(0, 52);
  display.println("SW4 = Stop");
  display.display();

  if (back()) {
    stopMotors();
    beepBack();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  MODE: LED COLOR PICKER
// =====================================================================
// SW1 = cycle channel (R->G->B), SW2 = increase value, SW3 = decrease,
// wait — clearer mapping below:
//   SW1 = -10 on selected channel
//   SW2 = +10 on selected channel
//   SW3 = switch channel (R/G/B) & SAVE when cycled past B
//   SW4 = back without extra save (still keeps last saved value)
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
  display.setCursor(0, 40);
  display.println("SW1=- SW2=+ SW3=Next");
  display.setCursor(0, 50);
  display.println("SW4=Save & Back");
  display.display();

  uint8_t* chan = (ledChannel == 0) ? &idleR : (ledChannel == 1) ? &idleG : &idleB;

  if (up())   { *chan = (uint8_t)max(0, *chan - 10); beep(1600,20); }
  if (down()) { *chan = (uint8_t)min(255, *chan + 10); beep(2000,20); }
  if (select()) { ledChannel = (ledChannel + 1) % 3; beep(2200,20); }

  setLED({idleR, idleG, idleB}); // live preview

  if (back()) {
    saveLedColor();
    beepBack();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  MODE: PID TUNE
// =====================================================================
// SW1/SW2 select parameter, SW3 decreases, hold pattern kept simple:
//   SW1 = cycle parameter (Kp -> Ki -> Kd -> Base Speed)
//   SW2 = decrease
//   SW3 = increase
//   SW4 = save & back
int pidParamIndex = 0; // 0=Kp,1=Ki,2=Kd,3=BaseSpeed
const char* pidParamName[4] = {"Kp", "Ki", "Kd", "BaseSpd"};

void pidTuneLoop() {
  oledHeader("PID TUNE");
  display.setCursor(0, 14);
  display.print("Param: "); display.println(pidParamName[pidParamIndex]);
  display.setCursor(0, 24);
  display.print("Kp="); display.println(Kp, 4);
  display.setCursor(0, 32);
  display.print("Ki="); display.println(Ki, 5);
  display.setCursor(0, 40);
  display.print("Kd="); display.println(Kd, 4);
  display.setCursor(0, 48);
  display.print("Base="); display.println(baseSpeed);
  display.display();

  if (up()) { pidParamIndex = (pidParamIndex + 1) % 4; beep(1800,20); }

  bool dec = down();
  bool inc = select();
  if (dec || inc) {
    int dir = inc ? 1 : -1;
    switch (pidParamIndex) {
      case 0: Kp += dir * 0.005f; Kp = max(0.0f, Kp); break;
      case 1: Ki += dir * 0.0001f; Ki = max(0.0f, Ki); break;
      case 2: Kd += dir * 0.02f; Kd = max(0.0f, Kd); break;
      case 3: baseSpeed += dir * 5; baseSpeed = constrain(baseSpeed, 0, 255); break;
    }
    beep(2000, 20);
  }

  if (back()) {
    savePID();
    beepBack();
    enterState(STATE_MENU);
  }
}

// =====================================================================
//  SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);

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

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // LED
  strip.begin();
  strip.setBrightness(80);
  strip.show();

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
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
  loadPID();

  applyModeLED();
  beepOK();
}

// =====================================================================
//  LOOP
// =====================================================================
void loop() {
  switch (currentState) {
    case STATE_MENU:        handleMenu();      break;
    case STATE_CALIBRATION: calibrationLoop(); break;
    case STATE_MOTOR_TEST:  motorTestLoop();   break;
    case STATE_LINE_FOLLOW: lineFollowLoop();  break;
    case STATE_LED_COLOR:   ledColorLoop();    break;
    case STATE_PID_TUNE:    pidTuneLoop();     break;
  }
  delay(5); // small yield
}
