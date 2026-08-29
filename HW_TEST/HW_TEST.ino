/*
  =====================================================================
  HW_TEST — plain hardware diagnostic, Serial only, no OLED/menu
  =====================================================================
  Purpose: check that every piece of hardware is alive and wired to the
  pin you think it is, before trusting FLF_V4.ino's more complex logic.
  Everything just prints to Serial (115200 baud) — open the Serial
  Monitor and watch.

  What it does:
    - On boot: flashes the LED white and beeps once (same self-test as
      FLF_V4), then prints "BOOT OK".
    - Every ~500ms: prints all 14 raw sensor values and both encoder
      counts on one line.
    - Any button press prints exactly which physical button it was
      (SW1/SW2/SW3/SW4 + its pin number) — this uses the RAW physical
      pins, no logical UP/DOWN/OK/BACK remapping, so it can't be
      confused by anything in FLF_V4's button-role layer.
    - Hold SW1: both motors forward at TEST_SPEED, prints once on press
      and once on release.
    - Hold SW2: both motors reverse at TEST_SPEED, same pattern.
    - Press SW3: cycles the LED through a fixed color list + beeps,
      prints the color name.
    - Press SW4: prints an immediate sensor+encoder snapshot on demand
      (in addition to the automatic 500ms line).

  Pins match FLF_V4.ino's current wiring table exactly (Waveshare
  ESP32-S3-Pico). If you rewire anything, update both files.
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

// ---- Debounced button edge detection ----
struct Btn { uint8_t pin; const char* name; bool last = HIGH; bool stable = HIGH; unsigned long changedAt = 0; };
Btn buttons[4] = { {SW1, "SW1 (pin 10)"}, {SW2, "SW2 (pin 11)"}, {SW3, "SW3 (pin 12)"}, {SW4, "SW4 (pin 13)"} };
const unsigned long DEBOUNCE_MS = 30;

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

void printSnapshot() {
  int sensorRaw[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(muxChannelForSensor[i]);
    sensorRaw[i] = analogRead(MUX_SIG);
  }
  Serial.print("SENSORS: ");
  for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(sensorRaw[i]); Serial.print(i < NUM_SENSORS - 1 ? "," : ""); }
  Serial.print("  | ENC L="); Serial.print(leftEncCount);
  Serial.print(" R="); Serial.println(rightEncCount);
}

void setup() {
  Serial.begin(115200);
  delay(300);

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
  // Each button's edge is read exactly once per loop and reused below --
  // calling checkButton() twice on the same button in one pass would
  // silently eat the second call's edge (state already advanced).
  int edge[4];
  for (int i = 0; i < 4; i++) {
    edge[i] = checkButton(buttons[i]);
    if (edge[i] == 1)  { Serial.print("BUTTON PRESSED:  "); Serial.println(buttons[i].name); }
    if (edge[i] == -1) { Serial.print("BUTTON RELEASED: "); Serial.println(buttons[i].name); }
  }

  // SW1/SW2 held: jog motors (raw hold state, not edge-based, so it
  // tracks release even if a release edge was missed by debounce timing).
  bool fwd = digitalRead(SW1) == LOW;
  bool rev = digitalRead(SW2) == LOW;
  static bool wasDriving = false;
  bool driving = fwd || rev;
  if (driving && !wasDriving) Serial.println(fwd ? "MOTOR TEST: FWD" : "MOTOR TEST: REV");
  else if (!driving && wasDriving) Serial.println("MOTOR TEST: STOP");
  wasDriving = driving;
  int cmd = fwd ? TEST_SPEED : (rev ? -TEST_SPEED : 0);
  setLeftMotor(cmd);
  setRightMotor(cmd);

  // SW3 press: cycle LED/buzzer test color
  if (edge[2] == 1) {
    NamedColor c = colors[colorIndex];
    setLED(c.r, c.g, c.b);
    beep(2000, 80);
    Serial.print("LED/BUZZER TEST: "); Serial.println(c.name);
    colorIndex = (colorIndex + 1) % (sizeof(colors) / sizeof(colors[0]));
  }

  // SW4 press: on-demand sensor+encoder snapshot
  if (edge[3] == 1) {
    Serial.println("--- SW4 manual snapshot ---");
    printSnapshot();
  }

  static unsigned long lastAutoPrint = 0;
  if (millis() - lastAutoPrint >= 500) {
    lastAutoPrint = millis();
    printSnapshot();
  }

  delay(5);
}
