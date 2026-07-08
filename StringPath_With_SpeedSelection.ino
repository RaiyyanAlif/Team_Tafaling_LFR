#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── OLED ─────────────────────────────
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── Motor Pins ───────────────────────
#define ENA 3
#define IN1 2
#define IN2 4

#define ENB 11
#define IN3 6
#define IN4 7

#define STBY 13

// ── Buttons ──────────────────────────
#define BTN_START 10
#define BTN_STOP  12

// ─────────────────────────────────────────────────────────────
//  MODE TOGGLE:
//    While robot is STOPPED (not running):
//      → Short press BTN_STOP (< LONG_PRESS_MS)   : toggle NORMAL ↔ INVERTED
//      → Long  press BTN_STOP (≥ LONG_PRESS_MS)   : enter PROGRAM PATH mode
//    While robot is RUNNING:
//      → Any  press BTN_STOP              : stop the robot (unchanged)
//
//  Toggling updates INVERTED_TRACK, JUNCTION_COUNT, and the OLED.
//  The new mode persists until toggled again or power-cycled.
// ─────────────────────────────────────────────────────────────

#define LONG_PRESS_MS 2000

// ── Sensors ──────────────────────────
const byte sensorPin[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

// ═══════════════════════════════════════════════════════════════
//  ★  TRACK MODE  —  AUTO DETECTED then MANUALLY TOGGLEABLE  ★
// ═══════════════════════════════════════════════════════════════
bool INVERTED_TRACK = false;   // auto-set in calibrate(); toggle via BTN_STOP

// ── PID CONFIG ───────────────────────
float Kp = 0.16;
float Kd = 0.34;

float lastError = 0;

// ── Speed ───────────────────────────
int BASE_SPEED   = 130;   // now selectable at startup from {120,140,160}
int MAX_SPEED    = 255;
int MOTOR_MIN    = 80;
int SEARCH_SPEED = 130;

// ═══════════════════════════════════════════════════════════════
//  ★  BASE SPEED MENU  —  select 120 / 140 / 160 at startup  ★
// ═══════════════════════════════════════════════════════════════
//    BTN_START (tap)         : cycle through the 3 speed options
//    BTN_STOP  (hold ≥2s)    : confirm selection, apply to BASE_SPEED
// ═══════════════════════════════════════════════════════════════
const int SPEED_OPTIONS[3] = {120, 140, 160};
int speedOptionIndex = 1;   // default -> 140 (middle option)

// ── Junction / branch handling ──────
int JUNCTION_TURN_SPEED = 150;
int JUNCTION_HOLD_MS    = 220;

// ── Junction sensor threshold ────────
int JUNCTION_COUNT = 5;   // updated by calibrate() and toggleMode()

// ── Calibration ─────────────────────
int sensorMin[8], sensorMax[8], sensorThresh[8];
char sensorStr[9];

// ── State ───────────────────────────
bool running = false;

// ── Lost-line memory ────────────────
int lastDir = 1;

// ── Stop bar ────────────────────────
bool allBlackForward = false;

// ═══════════════════════════════════════════════════════════════
//  ★★★  STRING PATH  —  pre-programmed junction directions  ★★★
// ═══════════════════════════════════════════════════════════════
//  Example: "LLFRL" → 1st junction = Left, 2nd = Left, 3rd = Front,
//           4th = Right, 5th = Left. After the string is used up,
//           the bot falls back to normal junction-turn logic.
// ═══════════════════════════════════════════════════════════════

#define PATH_MAX_LEN 40
char pathString[PATH_MAX_LEN + 1] = "";   // saved sequence, e.g. "LLFRL"
int  pathLength   = 0;                    // how many valid letters are saved
int  pathIndex    = 0;                    // how many letters consumed this run

// ═════════ FORWARD DECLARATIONS ═════════
void showReady();
void toggleMode();
void enterProgramPathMode();
void selectBaseSpeedMenu();
unsigned long waitReleaseAndMeasure(int pin);

// ─────────────────────────────────────────────────────────────
//  showReady()  — shared helper to (re)paint the READY screen
// ─────────────────────────────────────────────────────────────
void showReady() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("READY");
  display.println(INVERTED_TRACK ? "Mode: INVERTED" : "Mode: NORMAL");
  display.print("Speed: ");
  display.println(BASE_SPEED);
  display.print("Path: ");
  if (pathLength == 0) {
    display.println("(none)");
  } else {
    display.println(pathString);
  }
  display.println("Tap STOP=mode");
  display.println("Hold STOP=path edit");
  display.display();
}

// ─────────────────────────────────────────────────────────────
//  toggleMode()  — flip INVERTED_TRACK, update dependants,
//                  flash confirmation on OLED.
//  Called from loop() when BTN_STOP is short-pressed while stopped.
// ─────────────────────────────────────────────────────────────
void toggleMode() {
  INVERTED_TRACK = !INVERTED_TRACK;

  // Keep junction threshold in sync with the active mode
  JUNCTION_COUNT = INVERTED_TRACK ? 6 : 5;

  // ── Confirmation splash ──────────────────────────────────
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("MODE TOGGLED!");
  display.println("");
  display.println(INVERTED_TRACK ? ">> INVERTED <<" : ">>  NORMAL  <<");
  display.println("");
  display.println(INVERTED_TRACK ? "(white line)" : "(black line)");
  display.display();
  delay(1500);   // brief user feedback — only fires on manual toggle

  showReady();   // return to normal idle screen
}

// ─────────────────────────────────────────────────────────────
//  waitReleaseAndMeasure() — helper: assumes pin is already LOW,
//  blocks until release, returns hold duration in ms.
// ─────────────────────────────────────────────────────────────
unsigned long waitReleaseAndMeasure(int pin) {
  unsigned long pressStart = millis();
  while (digitalRead(pin) == LOW) {
    delay(5);
  }
  return millis() - pressStart;
}

// ─────────────────────────────────────────────────────────────
//  drawSpeedMenu() — draws the base-speed selection screen
// ─────────────────────────────────────────────────────────────
void drawSpeedMenu(int idx) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SELECT BASE SPEED");
  display.println("");

  for (int i = 0; i < 3; i++) {
    if (i == idx) display.print("> ");
    else          display.print("  ");
    display.println(SPEED_OPTIONS[i]);
  }

  display.println("");
  display.println("TAP D10 = next");
  display.println("HOLD D12 = select");
  display.display();
}

// ─────────────────────────────────────────────────────────────
//  selectBaseSpeedMenu() — interactive menu using:
//    BTN_START (D10) tap  : cycle 120 -> 140 -> 160 -> 120 ...
//    BTN_STOP  (D12) hold : confirm current option -> BASE_SPEED
//  Runs once at startup (after calibration, before READY screen).
// ─────────────────────────────────────────────────────────────
void selectBaseSpeedMenu() {
  drawSpeedMenu(speedOptionIndex);

  while (true) {

    // ── BTN_START tap: cycle through speed options ──────────
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);   // wait for release
      delay(30);   // debounce

      speedOptionIndex++;
      if (speedOptionIndex > 2) speedOptionIndex = 0;

      drawSpeedMenu(speedOptionIndex);
    }

    // ── BTN_STOP hold: confirm selection ────────────────────
    if (digitalRead(BTN_STOP) == LOW) {
      unsigned long heldMs = waitReleaseAndMeasure(BTN_STOP);
      delay(30);

      if (heldMs >= LONG_PRESS_MS) {
        BASE_SPEED = SPEED_OPTIONS[speedOptionIndex];

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("SPEED SET!");
        display.println("");
        display.print(">> ");
        display.print(BASE_SPEED);
        display.println(" <<");
        display.display();
        delay(1200);

        return;   // exit menu, continue to showReady()
      } else {
        // short press while in speed menu: ignored
        drawSpeedMenu(speedOptionIndex);
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  drawPathEditor() — draws the live edit screen:
//  shows the string built so far, the cursor slot, and the
//  letter currently being cycled for that slot.
// ─────────────────────────────────────────────────────────────
void drawPathEditor(char currentLetter, int slotNum) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("PROGRAM PATH");
  display.print("Slot ");
  display.print(slotNum + 1);
  display.print(": ");
  display.println(currentLetter == 0 ? '_' : currentLetter);

  display.println("");
  display.print("Str: ");
  // show saved letters so far, then the pending one in brackets
  for (int i = 0; i < slotNum && i < PATH_MAX_LEN; i++) {
    display.print(pathString[i]);
  }
  if (currentLetter != 0) {
    display.print('[');
    display.print(currentLetter);
    display.print(']');
  }
  display.println("");

  display.println("");
  display.println("TAP=cycle L/F/R");
  display.println("HOLD=confirm/next");
  display.display();
}

// ─────────────────────────────────────────────────────────────
//  enterProgramPathMode() — interactive editor using only
//  BTN_START (tap = cycle letter) and BTN_STOP (hold = confirm
//  slot & advance). Holding BTN_STOP on a BLANK slot ends and
//  saves the string.
// ─────────────────────────────────────────────────────────────
void enterProgramPathMode() {
  int slot = 0;
  char options[3] = {'L', 'F', 'R'};
  int optIndex = -1;          // -1 means "blank" (no letter chosen yet)
  char currentLetter = 0;     // 0 = blank

  drawPathEditor(currentLetter, slot);

  while (true) {

    // ── BTN_START tap: cycle L -> F -> R -> blank -> L ... ──
    if (digitalRead(BTN_START) == LOW) {
      // simple debounce / wait for release (short action only)
      while (digitalRead(BTN_START) == LOW) delay(5);
      delay(30);

      optIndex++;
      if (optIndex > 2) {
        optIndex = -1;       // wrap back to blank (means "end string here")
        currentLetter = 0;
      } else {
        currentLetter = options[optIndex];
      }

      drawPathEditor(currentLetter, slot);
    }

    // ── BTN_STOP hold: confirm current slot & advance ───────
    if (digitalRead(BTN_STOP) == LOW) {
      unsigned long heldMs = waitReleaseAndMeasure(BTN_STOP);
      delay(30);

      if (heldMs >= LONG_PRESS_MS) {

        if (currentLetter == 0) {
          // Confirmed a BLANK slot -> end of string, save & exit
          pathLength = slot;
          pathString[pathLength] = '\0';

          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("PATH SAVED!");
          display.println("");
          display.print("Str: ");
          display.println(pathLength == 0 ? "(none)" : pathString);
          display.display();
          delay(1500);

          showReady();
          return;   // exit editor

        } else {
          // Confirmed a real letter -> store it, move to next slot
          if (slot < PATH_MAX_LEN) {
            pathString[slot] = currentLetter;
            slot++;
          }
          optIndex = -1;
          currentLetter = 0;
          drawPathEditor(currentLetter, slot);
        }

      } else {
        // Short press of BTN_STOP while editing: ignored
        // (keeps behavior predictable; only long-press acts here)
        drawPathEditor(currentLetter, slot);
      }
    }
  }
}

// ═════════ MOTOR ═════════

int applyDeadzone(int spd) {
  if (spd == 0) return 0;
  return constrain(spd, MOTOR_MIN, MAX_SPEED);
}

void motorLeft(int spd, bool fwd) {
  digitalWrite(IN1, fwd);
  digitalWrite(IN2, !fwd);
  analogWrite(ENA, applyDeadzone(spd));
}

void motorRight(int spd, bool fwd) {
  digitalWrite(IN3, fwd);
  digitalWrite(IN4, !fwd);
  analogWrite(ENB, applyDeadzone(spd));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ═════════ SENSOR ═════════

void readSensors() {
  for (int i = 0; i < 8; i++) {
    int raw = analogRead(sensorPin[i]);

    if (INVERTED_TRACK) {
      // White line on black surface:
      // white = high reflectance = LOW  ADC value  → '1' (line)
      // black = low  reflectance = HIGH ADC value  → '0' (background)
      sensorStr[i] = (raw < sensorThresh[i]) ? '1' : '0';
    } else {
      // Black line on white surface:
      // black = low  reflectance = HIGH ADC value  → '1' (line)
      // white = high reflectance = LOW  ADC value  → '0' (background)
      sensorStr[i] = (raw > sensorThresh[i]) ? '1' : '0';
    }
  }
  sensorStr[8] = '\0';
}

float getPosition() {
  int sum = 0, count = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '1') {
      sum += i * 1000;
      count++;
    }
  }
  if (count == 0) return -1;
  return (float)sum / count;
}

// ─────────────────────────────────────────────────────────────
//  allBlack() — name kept for compatibility.
//  Checks: "are ALL sensors reading the LINE?"
//  Works identically for both NORMAL and INVERTED modes.
// ─────────────────────────────────────────────────────────────
bool allBlack() {
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '0') return false;
  }
  return true;
}

// ═════════ IMPROVED DETECTION ═════════

// ✅ Real junction — uses JUNCTION_COUNT (tunable per track mode)
// ═════════ IMPROVED JUNCTION DETECTION (shape-based) ═════════

// Cross junction: line present across almost the whole sensor bar
bool isCross() {
  int count = 0;
  for (int i = 0; i < 8; i++)
    if (sensorStr[i] == '1') count++;
  return count >= 7;   // 11111111 (or 7/8 with a little noise tolerance)
}

// Left T: outer-left block lit (0..4), outer-right clear (6,7)
// e.g. "11111000"
bool isLeftT() {
  int leftCount = 0, rightCount = 0;
  for (int i = 0; i <= 4; i++) if (sensorStr[i] == '1') leftCount++;
  for (int i = 6; i <= 7; i++) if (sensorStr[i] == '1') rightCount++;
  return (leftCount >= 4 && rightCount == 0);
}

// Right T: outer-right block lit (3..7), outer-left clear (0,1)
// e.g. "00011111"
bool isRightT() {
  int rightCount = 0, leftCount = 0;
  for (int i = 3; i <= 7; i++) if (sensorStr[i] == '1') rightCount++;
  for (int i = 0; i <= 1; i++) if (sensorStr[i] == '1') leftCount++;
  return (rightCount >= 4 && leftCount == 0);
}

// ✅ Junction = any recognized shape OR the old generic threshold as fallback
bool isJunction() {
  if (isCross() || isLeftT() || isRightT()) return true;

  int count = 0;
  for (int i = 0; i < 8; i++)
    if (sensorStr[i] == '1') count++;
  return (count >= JUNCTION_COUNT);
}

// ✅ Strong center detection
bool centerActive() {
  return (sensorStr[3] == '1' && sensorStr[4] == '1');
}

// ✅ Clean branch detection (ignore noise)
int sideBranch() {
  int leftCount = 0, rightCount = 0;

  for (int i = 0; i <= 2; i++)
    if (sensorStr[i] == '1') leftCount++;

  for (int i = 5; i <= 7; i++)
    if (sensorStr[i] == '1') rightCount++;

  if (rightCount >= 2) return 1;
  if (leftCount >= 2) return -1;

  return 0;
}

// ═════════ CALIBRATION ═════════

void calibrate() {
  Serial.begin(9600);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WHITE...");
  display.display();
  delay(2000);

  for (int i = 0; i < 8; i++)
    sensorMin[i] = analogRead(sensorPin[i]);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("BLACK...");
  display.display();
  delay(2000);

  for (int i = 0; i < 8; i++)
    sensorMax[i] = analogRead(sensorPin[i]);

  for (int i = 0; i < 8; i++)
    sensorThresh[i] = (sensorMin[i] + sensorMax[i]) / 2;

  // ★ AUTO DETECT INVERTED MODE ★
  int raw[8];
  for (int i = 0; i < 8; i++) {
    raw[i] = analogRead(sensorPin[i]);
  }

  // Outer sensors (S0,S1 and S6,S7) on BLACK?
  bool outerBlack = true;
  for (int i = 0; i <= 1; i++) {
    if (raw[i] <= sensorThresh[i]) outerBlack = false;
  }
  for (int i = 6; i <= 7; i++) {
    if (raw[i] <= sensorThresh[i]) outerBlack = false;
  }

  // Middle sensors (S3,S4) on WHITE?
  bool middleWhite = true;
  for (int i = 3; i <= 4; i++) {
    if (raw[i] >= sensorThresh[i]) middleWhite = false;
  }

  if (outerBlack && middleWhite) {
    INVERTED_TRACK = true;
    JUNCTION_COUNT = 6;
  } else {
    INVERTED_TRACK = false;
    JUNCTION_COUNT = 5;
  }

  // ── Serial verify ─────────────────────────────────────────
  Serial.println("=== Calibration Verify ===");
  Serial.println("Sen | White(Min) | Black(Max) | Thresh | OK?");
  for (int i = 0; i < 8; i++) {
    Serial.print("S");
    Serial.print(i);
    Serial.print("  |    ");
    Serial.print(sensorMin[i]);
    Serial.print("     |    ");
    Serial.print(sensorMax[i]);
    Serial.print("     |   ");
    Serial.print(sensorThresh[i]);
    Serial.print("  |  ");
    Serial.println(sensorMin[i] < sensorMax[i] ? "OK" : "!! REVERSED !!");
  }
  Serial.println("==========================");
  Serial.print("Track mode: ");
  Serial.println(INVERTED_TRACK ? "INVERTED (white line)" : "NORMAL (black line)");
}

// ═════════ SETUP ═════════

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;);
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.display();

  calibrate();

  // ★ NEW: select base speed (120/140/160) before going READY ★
  selectBaseSpeedMenu();

  showReady();   // paint READY + mode + speed + toggle hint
}

// ═════════ LOOP ═════════

void loop() {

  // ── START button ──────────────────────────────────────────
  if (digitalRead(BTN_START) == LOW) {
    running = true;
    pathIndex = 0;          // reset string-path progress for the new run
    delay(200);
  }

  // ── STOP button ───────────────────────────────────────────
  if (digitalRead(BTN_STOP) == LOW) {

    if (running) {
      // ── Robot is running → STOP immediately ───────────────
      running = false;
      stopMotors();
      delay(200);   // debounce
      showReady();

    } else {
      // ── Robot is already stopped → measure press length ───
      unsigned long pressDuration = waitReleaseAndMeasure(BTN_STOP);

      if (pressDuration >= LONG_PRESS_MS) {
        // Long press while stopped = enter Program Path editor
        enterProgramPathMode();
      } else {
        // Short press while stopped = toggle NORMAL/INVERTED
        toggleMode();
      }
    }
  }

  if (!running) return;

  readSensors();

  // ✅ STOP BAR
  if (allBlack()) {
    if (!allBlackForward) {
      motorLeft(BASE_SPEED, true);
      motorRight(BASE_SPEED, true);
      delay(150);
      allBlackForward = true;
    }

    readSensors();
    if (allBlack()) {
      stopMotors();
      running = false;
      showReady();
      return;
    }
  } else {
    allBlackForward = false;
  }

  // 🔀 FULL JUNCTION
  if (isJunction()) {

    // ═══ STRING PATH OVERRIDE ═══
    // If we still have un-consumed letters in the programmed
    // path, use that direction instead of the default turn.
    if (pathIndex < pathLength) {
      char dir = pathString[pathIndex];
      pathIndex++;

      if (dir == 'L') {
        motorLeft(JUNCTION_TURN_SPEED, false);
        motorRight(JUNCTION_TURN_SPEED, true);
        delay(JUNCTION_HOLD_MS);
        lastDir = -1;
      } else if (dir == 'R') {
        motorLeft(JUNCTION_TURN_SPEED, true);
        motorRight(JUNCTION_TURN_SPEED, false);
        delay(JUNCTION_HOLD_MS);
        lastDir = 1;
      } else { // 'F' — go straight through the junction
        motorLeft(BASE_SPEED, true);
        motorRight(BASE_SPEED, true);
        delay(JUNCTION_HOLD_MS);
      }
      return;
    }

    // ── Default junction behavior (string exhausted / none set) ──
    motorLeft(JUNCTION_TURN_SPEED, true);
    motorRight(JUNCTION_TURN_SPEED, false);
    delay(JUNCTION_HOLD_MS);
    lastDir = 1;
    return;
  }

  // 🔀 CENTER + SIDE BRANCH
  if (centerActive()) {
    int branch = sideBranch();

    if (branch == 1) {
      motorLeft(JUNCTION_TURN_SPEED, true);
      motorRight(JUNCTION_TURN_SPEED, false);
      delay(JUNCTION_HOLD_MS);
      lastDir = 1;
      return;
    }

    if (branch == -1) {
      motorLeft(JUNCTION_TURN_SPEED, false);
      motorRight(JUNCTION_TURN_SPEED, true);
      delay(JUNCTION_HOLD_MS);
      lastDir = -1;
      return;
    }
  }

  float pos = getPosition();

  // ✅ STRONG LOST LINE RECOVERY
  if (pos == -1) {

    if (lastDir > 0) {
      motorLeft(SEARCH_SPEED, true);
      motorRight(SEARCH_SPEED, false);
    } else {
      motorLeft(SEARCH_SPEED, false);
      motorRight(SEARCH_SPEED, true);
    }

    return;
  }

  // 🎯 PID
  float error = pos - 3500;

  // ✅ Direction memory — only update on significant deviation
  if (error > 600) lastDir = 1;
  else if (error < -600) lastDir = -1;

  float derivative = error - lastError;
  float correction  = Kp * error + Kd * derivative;
  lastError = error;

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  motorLeft(leftSpeed,  true);
  motorRight(rightSpeed, true);
}
