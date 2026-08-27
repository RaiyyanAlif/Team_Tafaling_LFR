#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ── OLED ─────────────────────────────
// 1.3" I2C OLED, 128x64, SSH1106 controller
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

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

#define LONG_PRESS_MS 600

// ── Sensors ──────────────────────────
const byte sensorPin[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

// ═══════════════════════════════════════════════════════════════
//  ★  TRACK MODE  —  AUTO DETECTED then MANUALLY TOGGLEABLE  ★
// ═══════════════════════════════════════════════════════════════
bool INVERTED_TRACK = false;   // auto-set in calibrate(); toggle via BTN_STOP

// ── PID CONFIG ───────────────────────
float Kp = 0.06;
float Kd = 0;

float lastError = 0;

// ── Speed ───────────────────────────
int BASE_SPEED   = 130;   // now selectable at startup from SPEED_OPTIONS
int MAX_SPEED    = 255;
int MOTOR_MIN    = 80;
int SEARCH_SPEED = 130;

// ═══════════════════════════════════════════════════════════════
//  ★  BASE SPEED MENU  —  select 110/120/130/140/150/160 at startup  ★
// ═══════════════════════════════════════════════════════════════
//    BTN_START (tap)         : cycle through the speed options
//    BTN_STOP  (hold ≥2s)    : confirm selection, apply to BASE_SPEED
// ═══════════════════════════════════════════════════════════════
const int SPEED_OPTIONS[6] = {110, 120, 130, 140, 150, 160};
const int SPEED_OPTIONS_COUNT = 6;
int speedOptionIndex = 2;   // default -> 130

// ── Junction / branch handling ──────
int JUNCTION_TURN_SPEED = 150;
int JUNCTION_HOLD_MS    = 180;

// After a FRONT decision only: a few milliseconds to leave the tape bar.
// There is NO forward seek before the decision.
int JUNCTION_EXIT_SPEED = 80;
int JUNCTION_EXIT_MS    = 12;

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

// Ignore further junction decisions until outer sensors are clear.
bool junctionLatched = false;
char lastJunctionDir = 'F';
unsigned long junctionLatchMs = 0;

// ═══════════════════════════════════════════════════════════════
//  ★★★  STRING PATH  —  pre-programmed junction directions  ★★★
// ═══════════════════════════════════════════════════════════════
//  Example: "LLFRL" → 1st junction = Left, 2nd = Left, 3rd = Front,
//           4th = Right, 5th = Left. After the string is used up,
//           the bot falls back to the PRIORITY logic (see below).
// ═══════════════════════════════════════════════════════════════

#define PATH_MAX_LEN 40
char pathString[PATH_MAX_LEN + 1] = "";   // saved sequence, e.g. "LLFRL"
int  pathLength   = 0;                    // how many valid letters are saved
int  pathIndex    = 0;                    // how many letters consumed this run

// ═══════════════════════════════════════════════════════════════
//  ★★★  PATH SOURCE  —  Checkpoints (predefined) vs Manual  ★★★
// ═══════════════════════════════════════════════════════════════
//    PATH_SOURCE_CHECKPOINT : pathString copied from a predefined
//                              checkpoint constant below.
//    PATH_SOURCE_MANUAL     : pathString built via the button-press
//                              editor (enterProgramPathMode()).
// ═══════════════════════════════════════════════════════════════
enum PathSource { PATH_SOURCE_CHECKPOINT, PATH_SOURCE_MANUAL };
PathSource pathSource = PATH_SOURCE_MANUAL;

// ── Predefined checkpoint strings (edit these as needed) ────────
const char* CHECKPOINTS[8] = {
  "FFFLLRRLR",   // Checkpoint 1
  "RLLFFFRL",    // Checkpoint 2
  "FRLFRLFR",    // Checkpoint 3
  "LLLRRRFF",    // Checkpoint 4
  "RRFFLLRF",    // Checkpoint 5
  "FFLRFLRL",    // Checkpoint 6
  "LRLRLRLR",    // Checkpoint 7
  "RFLFRLFL"     // Checkpoint 8
};
const int CHECKPOINT_COUNT = 8;
int checkpointIndex = 0;

// ═══════════════════════════════════════════════════════════════
//  ★★★  PRIORITY MODE  —  fallback turn direction after the
//        programmed path string is exhausted (or unset)  ★★★
// ═══════════════════════════════════════════════════════════════
//    PRIORITY_LEFT  : FRONT > LEFT > RIGHT
//    PRIORITY_RIGHT : FRONT > RIGHT > LEFT
//  Junctions are classified first; these rules never run on a
//  normal single-line follow state.
// ═══════════════════════════════════════════════════════════════
enum PriorityMode { PRIORITY_LEFT, PRIORITY_RIGHT };
PriorityMode priorityMode = PRIORITY_RIGHT;

// ═════════ FORWARD DECLARATIONS ═════════
void showReady();
void toggleMode();
void enterProgramPathMode();
void selectBaseSpeedMenu();
void selectPathSourceMenu();
void selectCheckpointMenu();
void selectPriorityMenu();
unsigned long waitReleaseAndMeasure(int pin);
int  sensorOnCount();
void readJunctionFlags(bool &L, bool &R, bool &F);
char classifyJunctionAction();
void executeJunctionMove(char dir);
void reacquireAfterJunction(char dir);

// ─────────────────────────────────────────────────────────────
//  helper: text for priority mode
// ─────────────────────────────────────────────────────────────
const char* priorityName(PriorityMode p) {
  if (p == PRIORITY_LEFT) return "LEFT";
  return "RIGHT";
}

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
  display.print("Priority: ");
  display.println(priorityName(priorityMode));
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

  for (int i = 0; i < SPEED_OPTIONS_COUNT; i++) {
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
//    BTN_START (D10) tap  : cycle through SPEED_OPTIONS
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
      if (speedOptionIndex >= SPEED_OPTIONS_COUNT) speedOptionIndex = 0;

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

        return;   // exit menu, continue to next setup step
      } else {
        // short press while in speed menu: ignored
        drawSpeedMenu(speedOptionIndex);
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  drawPathSourceMenu() — Checkpoints vs Manual
// ─────────────────────────────────────────────────────────────
void drawPathSourceMenu(PathSource sel) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("CHOOSE PATH SOURCE");
  display.println("");

  display.print(sel == PATH_SOURCE_CHECKPOINT ? "> " : "  ");
  display.println("Checkpoints");

  display.print(sel == PATH_SOURCE_MANUAL ? "> " : "  ");
  display.println("Manual");

  display.println("");
  display.println("TAP D10 = next");
  display.println("HOLD D12 = select");
  display.display();
}

// ─────────────────────────────────────────────────────────────
//  selectPathSourceMenu() — choose between Checkpoints & Manual.
//  Runs at startup, after speed menu, before READY screen.
// ─────────────────────────────────────────────────────────────
void selectPathSourceMenu() {
  drawPathSourceMenu(pathSource);

  while (true) {

    // ── BTN_START tap: toggle Checkpoints <-> Manual ────────
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);
      delay(30);

      pathSource = (pathSource == PATH_SOURCE_CHECKPOINT) ? PATH_SOURCE_MANUAL : PATH_SOURCE_CHECKPOINT;
      drawPathSourceMenu(pathSource);
    }

    // ── BTN_STOP hold: confirm selection ────────────────────
    if (digitalRead(BTN_STOP) == LOW) {
      unsigned long heldMs = waitReleaseAndMeasure(BTN_STOP);
      delay(30);

      if (heldMs >= LONG_PRESS_MS) {
        return;   // move on to the relevant sub-menu in setup()
      } else {
        drawPathSourceMenu(pathSource);
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  drawCheckpointMenu() — shows the checkpoint index + preview
// ─────────────────────────────────────────────────────────────
void drawCheckpointMenu(int idx) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("CHECKPOINT ");
  display.println(idx + 1);
  display.println("");
  display.println(CHECKPOINTS[idx]);
  display.println("");
  display.println("TAP D10 = next");
  display.println("HOLD D12 = select");
  display.display();
}

// ─────────────────────────────────────────────────────────────
//  selectCheckpointMenu() — cycle through CHECKPOINTS[0..7] and
//  confirm one, copying it into pathString / pathLength.
// ─────────────────────────────────────────────────────────────
void selectCheckpointMenu() {
  drawCheckpointMenu(checkpointIndex);

  while (true) {

    // ── BTN_START tap: next checkpoint ──────────────────────
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);
      delay(30);

      checkpointIndex++;
      if (checkpointIndex >= CHECKPOINT_COUNT) checkpointIndex = 0;

      drawCheckpointMenu(checkpointIndex);
    }

    // ── BTN_STOP hold: confirm this checkpoint ──────────────
    if (digitalRead(BTN_STOP) == LOW) {
      unsigned long heldMs = waitReleaseAndMeasure(BTN_STOP);
      delay(30);

      if (heldMs >= LONG_PRESS_MS) {
        strncpy(pathString, CHECKPOINTS[checkpointIndex], PATH_MAX_LEN);
        pathString[PATH_MAX_LEN] = '\0';
        pathLength = strlen(pathString);
        pathIndex = 0;

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("CHECKPOINT LOADED!");
        display.println("");
        display.println(pathString);
        display.display();
        delay(1500);

        return;
      } else {
        drawCheckpointMenu(checkpointIndex);
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  drawPriorityMenu() — Left / Right selection
// ─────────────────────────────────────────────────────────────
void drawPriorityMenu(PriorityMode sel) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("CHOOSE PRIORITY");
  display.println("");

  display.print(sel == PRIORITY_LEFT ? "> " : "  ");
  display.println("Left");

  display.print(sel == PRIORITY_RIGHT ? "> " : "  ");
  display.println("Right");

  display.println("");
  display.println("TAP D10 = next");
  display.println("HOLD D12 = select");
  display.display();
}

// ─────────────────────────────────────────────────────────────
//  selectPriorityMenu() — LEFT or RIGHT for the whole run.
//  Used once the programmed path string runs out (or is empty).
// ─────────────────────────────────────────────────────────────
void selectPriorityMenu() {
  drawPriorityMenu(priorityMode);

  while (true) {

    // ── BTN_START tap: toggle Left <-> Right ────────────────
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);
      delay(30);

      priorityMode = (priorityMode == PRIORITY_LEFT) ? PRIORITY_RIGHT : PRIORITY_LEFT;

      drawPriorityMenu(priorityMode);
    }

    // ── BTN_STOP hold: confirm selection ────────────────────
    if (digitalRead(BTN_STOP) == LOW) {
      unsigned long heldMs = waitReleaseAndMeasure(BTN_STOP);
      delay(30);

      if (heldMs >= LONG_PRESS_MS) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("PRIORITY SET!");
        display.println("");
        display.print(">> ");
        display.print(priorityName(priorityMode));
        display.println(" <<");
        display.display();
        delay(1200);

        return;
      } else {
        drawPriorityMenu(priorityMode);
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
//  saves the string. Used when pathSource == PATH_SOURCE_MANUAL.
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
          pathSource = PATH_SOURCE_MANUAL;

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

// ═════════ JUNCTION DETECTION (sensor-pattern, not delays) ═════════
//
//   L = s[0] OR s[1]     outer left
//   R = s[6] OR s[7]     outer right
//   F = s[3] OR s[4]     front / center
//
// Exclusive if/else-if. Cases 7 and 8 return 0 → normal PID / lost-line.
// A slight PID offset that only lights s[1] (not s[0]) is NOT a left T.

int sensorOnCount() {
  int n = 0;
  for (int i = 0; i < 8; i++)
    if (sensorStr[i] == '1') n++;
  return n;
}

void readJunctionFlags(bool &L, bool &R, bool &F) {
  L = (sensorStr[0] == '1' || sensorStr[1] == '1');
  R = (sensorStr[6] == '1' || sensorStr[7] == '1');
  F = (sensorStr[3] == '1' || sensorStr[4] == '1');
}

bool outersClear() {
  return (sensorStr[0] == '0' && sensorStr[1] == '0'
          && sensorStr[6] == '0' && sensorStr[7] == '0');
}

// Returns 'F', 'L', 'R', or 0 if this is NOT a junction.
char classifyJunctionAction() {
  bool L, R, F;
  readJunctionFlags(L, R, F);
  int n = sensorOnCount();
  bool rightOuterOff = (sensorStr[6] == '0' && sensorStr[7] == '0');
  bool leftOuterOff  = (sensorStr[0] == '0' && sensorStr[1] == '0');

  // 7. Front-only line → PID, never a junction
  if (!L && !R && F) return 0;

  // 8. Nothing → lost-line recovery, never a junction
  if (!L && !R && !F) return 0;

  // 1. CROSS  L=1 R=1 F=1  → FRONT (both priorities)
  if (L && R && F && n >= 6) return 'F';

  // 3. LEFT/RIGHT CHOICE  L=1 R=1 F=0  → priority turn
  if (L && R && !F && n >= 5) {
    return (priorityMode == PRIORITY_LEFT) ? 'L' : 'R';
  }

  // 2. LEFT T  L=1 R=0 F=1  → FRONT (both priorities)
  //    True outer s[0] (or a fat count) so PID-left-offset is not a T.
  if (L && !R && F && rightOuterOff && n >= 3) {
    if (sensorStr[0] == '1' || n >= 5) return 'F';
  }

  // 6. RIGHT T  L=0 R=1 F=1  → FRONT (both priorities)
  if (!L && R && F && leftOuterOff && n >= 3) {
    if (sensorStr[7] == '1' || n >= 5) return 'F';
  }

  // 4. LEFT 90  L=1 R=0 F=0  → LEFT (both priorities)
  if (L && !R && !F && rightOuterOff) {
    if (sensorStr[0] == '1' && sensorStr[1] == '1') return 'L';
    if (n >= 2 && sensorStr[0] == '1') return 'L';
  }

  // 5. RIGHT 90  L=0 R=1 F=0  → RIGHT (both priorities)
  if (!L && R && !F && leftOuterOff) {
    if (sensorStr[6] == '1' && sensorStr[7] == '1') return 'R';
    if (n >= 2 && sensorStr[7] == '1') return 'R';
  }

  return 0;
}

void executeJunctionMove(char dir) {
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
  } else {
    // FRONT: tiny exit AFTER the decision, not a pre-decision seek
    motorLeft(JUNCTION_EXIT_SPEED, true);
    motorRight(JUNCTION_EXIT_SPEED, true);
    delay(JUNCTION_EXIT_MS);
  }
  stopMotors();
}

// After L/R, spin until the center line is seen. FRONT does not creep here.
void reacquireAfterJunction(char dir) {
  if (dir == 'F') return;

  unsigned long t0 = millis();
  while (millis() - t0 < 400) {
    readSensors();
    bool F = (sensorStr[3] == '1' || sensorStr[4] == '1');
    if (F && classifyJunctionAction() == 0) {
      stopMotors();
      return;
    }

    if (dir == 'L') {
      motorLeft(SEARCH_SPEED, false);
      motorRight(SEARCH_SPEED, true);
    } else {
      motorLeft(SEARCH_SPEED, true);
      motorRight(SEARCH_SPEED, false);
    }
  }
  stopMotors();
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

  // SSH1106 OLED init (I2C address 0x3C, as per module datasheet)
  if (!display.begin(0x3C, true)) {
    for (;;);
  }

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Calibrating...");
  display.display();

  calibrate();

  // ★ select base speed (110/120/130/140/150/160) ★
  selectBaseSpeedMenu();

  // ★ priority — Left / Right — immediately after speed ★
  selectPriorityMenu();

  // ★ path source — Checkpoints or Manual ★
  selectPathSourceMenu();

  if (pathSource == PATH_SOURCE_CHECKPOINT) {
    selectCheckpointMenu();     // pick one of the predefined strings
  } else {
    enterProgramPathMode();     // build the string by hand, as before
  }

  showReady();   // paint READY + mode + speed + path + priority
}

// ═════════ LOOP ═════════

void loop() {

  // ── START button ──────────────────────────────────────────
  if (digitalRead(BTN_START) == LOW) {
    running = true;
    pathIndex = 0;          // reset string-path progress for the new run
    junctionLatched = false;
    lastJunctionDir = 'F';
    delay(200);
  }

  // ── STOP button ───────────────────────────────────────────
  if (digitalRead(BTN_STOP) == LOW) {

    if (running) {
      // ── Robot is running → STOP immediately ───────────────
      running = false;
      stopMotors();
      junctionLatched = false;
      delay(200);   // debounce
      showReady();

    } else {
      // ── Robot is already stopped → measure press length ───
      unsigned long pressDuration = waitReleaseAndMeasure(BTN_STOP);

      if (pressDuration >= LONG_PRESS_MS) {
        // Long press while stopped = enter Manual Program Path editor
        pathSource = PATH_SOURCE_MANUAL;
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
      junctionLatched = false;
      showReady();
      return;
    }
  } else {
    allBlackForward = false;
  }

  // Leave cooldown only after outer sensors are off the junction bar
  if (junctionLatched && outersClear()) {
    junctionLatched = false;
  }

  char jAction = 0;
  if (!junctionLatched) {
    jAction = classifyJunctionAction();
  }

  // 🔀 JUNCTION — decide from the current pattern. No pre-decision seek.
  if (jAction != 0) {
    stopMotors();   // freeze PID immediately; do not roll into the junction

    char dir;
    if (pathIndex < pathLength) {
      dir = pathString[pathIndex];
      pathIndex++;
    } else {
      dir = jAction;
    }

    executeJunctionMove(dir);
    reacquireAfterJunction(dir);
    lastJunctionDir = dir;
    junctionLatched = true;
    junctionLatchMs = millis();
    return;
  }

  // After GO FRONT: brief straight so PID does not pull into the side
  // branch. Hard-capped so the chassis cannot drive through the junction.
  if (junctionLatched && lastJunctionDir == 'F' && !outersClear()
      && (millis() - junctionLatchMs) < 80) {
    motorLeft(BASE_SPEED, true);
    motorRight(BASE_SPEED, true);
    return;
  }

  float pos = getPosition();

  // ✅ STRONG LOST LINE RECOVERY
  if (pos == -1) {

    if (priorityMode == PRIORITY_LEFT) {
      motorLeft(SEARCH_SPEED, false);
      motorRight(SEARCH_SPEED, true);
    } else {
      motorLeft(SEARCH_SPEED, true);
      motorRight(SEARCH_SPEED, false);
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
