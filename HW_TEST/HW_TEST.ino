/*
  =====================================================================
  HW_TEST — plain hardware diagnostic: Serial AND OLED, no menu
  =====================================================================
  Purpose: check that every piece of hardware is alive and wired to the
  pin you think it is, before trusting FLF_V4.ino's more complex logic.
  Everything that prints to Serial (115200 baud) is ALSO shown live on
  the OLED — sensor bars, encoder counts, the most recent button event,
  and current motor/LED test status — so you don't need a laptop open
  to see it working.

  What it does:
    - On boot: flashes the LED white and beeps once (same self-test as
      FLF_V4), then prints "BOOT OK".
    - Every ~500ms: prints/shows all 14 raw sensor values (live bars on
      OLED) and both encoder counts.
    - Any button press/release shows + prints exactly which physical
      button it was (SW1/SW2/SW3/SW4 + its pin number) — RAW physical
      pins, no logical UP/DOWN/OK/BACK remapping, so it can't be
      confused by anything in FLF_V4's button-role layer.
    - Hold SW1: both motors forward at TEST_SPEED.
    - Hold SW2: both motors reverse at TEST_SPEED.
    - Press SW3: cycles the LED through a fixed color list + beeps.
    - Press SW4: on-demand sensor+encoder snapshot (Serial + OLED).

  Pins match FLF_V4.ino's current wiring table exactly (Waveshare
  ESP32-S3-Pico). If you rewire anything, update both files.
  =====================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// ---- Multiplexer (CD74HC4067) ----
#define MUX_SIG   1
#define MUX_S3    2
#define MUX_S2    4
#define MUX_S1    5
#define MUX_S0    6
#define NUM_SENSORS 14
const uint8_t muxChannelForSensor[NUM_SENSORS] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13};

// ---- Buttons (raw physical pins, no remapping) ----
#define SW1 10
#define SW2 11
#define SW3 12
#define SW4 13

// ---- Motors (BTS7960) ----
#define R_RPWM 15
#define R_LPWM 16
#define L_LPWM 33
#define L_RPWM 34
#define PWM_FREQ 20000
#define PWM_RES  8
const int TEST_SPEED = 150;

// ---- Encoders ----
#define L_ENC_A 17
#define L_ENC_B 18
#define R_ENC_A 35
#define R_ENC_B 36

// ---- LED (WS2812) ----
#define LED_PIN 7
#define NUM_LEDS 2
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---- Buzzer ----
#define BUZZER_PIN 37

// ---- OLED (I2C) ----
#define OLED_SDA  8
#define OLED_SCL  9
#define OLED_ADDR 0x3C
#define OLED_W    128
#define OLED_H    64
Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1);
bool oledOk = false; // if init fails, everything still works via Serial only

// ---- Debounced button edge detection ----
struct Btn { uint8_t pin; const char* name; bool last = HIGH; bool stable = HIGH; unsigned long changedAt = 0; };
Btn buttons[4] = { {SW1, "SW1 (pin 10)"}, {SW2, "SW2 (pin 11)"}, {SW3, "SW3 (pin 12)"}, {SW4, "SW4 (pin 13)"} };
const unsigned long DEBOUNCE_MS = 30;

// =====================================================================
volatile long leftEncCount = 0;
volatile long rightEncCount = 0;
void IRAM_ATTR leftEncoderISR()  { leftEncCount  += digitalRead(L_ENC_B) ? 1 : -1; }
void IRAM_ATTR rightEncoderISR() { rightEncCount += digitalRead(R_ENC_B) ? 1 : -1; }

void beep(int freq, int dur) {
  ledcWriteTone(BUZZER_PIN, freq);
  delay(dur);
  ledcWriteTone(BUZZER_PIN, 0);
}

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}

void selectMuxChannel(uint8_t ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
  digitalWrite(MUX_S3, (ch >> 3) & 1);
  delayMicroseconds(8);
}

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(L_RPWM, speed); ledcWrite(L_LPWM, 0); }
  else            { ledcWrite(L_RPWM, 0); ledcWrite(L_LPWM, -speed); }
}
void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) { ledcWrite(R_RPWM, speed); ledcWrite(R_LPWM, 0); }
  else            { ledcWrite(R_RPWM, 0); ledcWrite(R_LPWM, -speed); }
}
void stopMotors() { setLeftMotor(0); setRightMotor(0); }



// Returns 1 on press edge, -1 on release edge, 0 otherwise
int checkButton(Btn &b) {
  bool reading = digitalRead(b.pin);
  if (reading != b.last) b.changedAt = millis();
  int edge = 0;
  if ((millis() - b.changedAt) > DEBOUNCE_MS && reading != b.stable) {
    b.stable = reading;
    edge = (b.stable == LOW) ? 1 : -1;
  }
  b.last = reading;
  return edge;
}

// ---- LED color test cycle (SW3) ----
struct NamedColor { const char* name; uint8_t r, g, b; };
NamedColor colors[] = {
  {"RED", 255, 0, 0}, {"GREEN", 0, 255, 0}, {"BLUE", 0, 0, 255},
  {"WHITE", 255, 255, 255}, {"OFF", 0, 0, 0}
};
int colorIndex = 0;

// Live status shown on the OLED and echoed in Serial snapshots -- updated
// wherever the corresponding event happens in loop().
int sensorRaw[NUM_SENSORS];
String lastEvent = "-";
String motorStatus = "STOP";
String ledStatus = "-";

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(muxChannelForSensor[i]);
    sensorRaw[i] = analogRead(MUX_SIG);
  }
}

void printSnapshot() {
  Serial.print("SENSORS: ");
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(sensorRaw[i]); Serial.print(i < NUM_SENSORS - 1 ? "," : ""); }
  Serial.print("  | ENC L="); Serial.print(leftEncCount);
  Serial.print(" R="); Serial.println(rightEncCount);
}

void drawDisplay() {
  if (!oledOk) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("HW TEST");
  display.drawLine(0, 10, OLED_W, 10, SSD1306_WHITE);

  // Live raw sensor bars (0-4095, uncalibrated -- just proves each sensor
  // is reading something and changes when you wave a hand/line under it).
  const int barTop = 12, barBottom = 38, barMaxH = barBottom - barTop;
  int slot = OLED_W / NUM_SENSORS; if (slot < 6) slot = 6;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int h = map(constrain(sensorRaw[i], 0, 4095), 0, 4095, 0, barMaxH);
    int x = i * slot, w = slot - 2; if (w < 2) w = 2;
    display.drawRect(x, barTop, w, barMaxH, SSD1306_WHITE);
    display.fillRect(x + 1, barBottom - h, max(w - 2, 1), h, SSD1306_WHITE);
  }

  display.setCursor(0, 41);
  display.print("ENC L:"); display.print(leftEncCount);
  display.setCursor(70, 41);
  display.print("R:"); display.println(rightEncCount);

  display.setCursor(0, 51);
  display.print("Last: "); display.println(lastEvent);

  display.setCursor(0, 59);
  display.print("Mot:"); display.print(motorStatus);
  display.print(" LED:"); display.print(ledStatus);

  display.display();
}

void setup() {
  Serial.begin(115200);
  // Wait up to 3s for USB-CDC Serial to connect (ESP32-S3 HWCDC)
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) delay(10);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  analogReadResolution(12);

  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  pinMode(SW4, INPUT_PULLUP);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);

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

  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);
  oledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOk) { display.clearDisplay(); display.display(); }
  else Serial.println("OLED init FAILED -- check wiring/address. Serial output still works.");

  // Boot self-test
  setLED(255, 255, 255);
  beep(1800, 150);
  setLED(0, 0, 0);

  Serial.println();
  Serial.println("=== HW_TEST BOOT OK ===");
  Serial.println("Hold SW1 = motors FWD | Hold SW2 = motors REV | SW3 = LED/buzzer cycle | SW4 = manual snapshot");
  Serial.println("Sensor+encoder line prints automatically every ~500ms.");
  Serial.println("========================");
}

void loop() {
  readSensors(); // fresh every loop -- feeds both the live OLED bars and printSnapshot()

  // Each button's edge is read exactly once per loop and reused below --
  // calling checkButton() twice on the same button in one pass would
  // silently eat the second call's edge (state already advanced).
  int edge[4];
  for (int i = 0; i < 4; i++) {
    edge[i] = checkButton(buttons[i]);
    if (edge[i] == 1)  { lastEvent = String("PRESSED ") + buttons[i].name; Serial.print("BUTTON PRESSED:  "); Serial.println(buttons[i].name); }
    if (edge[i] == -1) { lastEvent = String("RELEASED ") + buttons[i].name; Serial.print("BUTTON RELEASED: "); Serial.println(buttons[i].name); }
  }

  // SW1/SW2 held: jog motors (raw hold state, not edge-based, so it
  // tracks release even if a release edge was missed by debounce timing).
  bool fwd = digitalRead(SW1) == LOW;
  bool rev = digitalRead(SW2) == LOW;
  static bool wasDriving = false;
  bool driving = fwd || rev;
  if (driving && !wasDriving) { motorStatus = fwd ? "FWD" : "REV"; Serial.println(fwd ? "MOTOR TEST: FWD" : "MOTOR TEST: REV"); }
  else if (!driving && wasDriving) { motorStatus = "STOP"; Serial.println("MOTOR TEST: STOP"); }
  wasDriving = driving;
  int cmd = fwd ? TEST_SPEED : (rev ? -TEST_SPEED : 0);
  setLeftMotor(cmd);
  setRightMotor(cmd);

  // SW3 press: cycle LED/buzzer test color
  if (edge[2] == 1) {
    NamedColor c = colors[colorIndex];
    setLED(c.r, c.g, c.b);
    beep(2000, 80);
    ledStatus = c.name;
    lastEvent = String("LED/BUZZER: ") + c.name;
    Serial.print("LED/BUZZER TEST: "); Serial.println(c.name);
    colorIndex = (colorIndex + 1) % (sizeof(colors) / sizeof(colors[0]));
  }

  // SW4 press: on-demand sensor+encoder snapshot
  if (edge[3] == 1) {
    lastEvent = "Manual snapshot";
    Serial.println("--- SW4 manual snapshot ---");
    printSnapshot();
  }

  static unsigned long lastAutoPrint = 0;
  if (millis() - lastAutoPrint >= 500) {
    lastAutoPrint = millis();
    printSnapshot();
  }

  static unsigned long lastDraw = 0;
  if (millis() - lastDraw >= 100) { // ~10Hz OLED refresh
    lastDraw = millis();
    drawDisplay();
  }

  delay(5);
}
