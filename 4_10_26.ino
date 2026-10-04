#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =================================================================
//  *** CUSTOM TYPES DECLARED FIRST ***
//  IMPORTANT: Arduino IDE 1.6.x's auto-prototype generator inserts
//  forward declarations for every function near the TOP of the
//  file (right after includes), before it has seen any of your own
//  enum/struct definitions further down. If a function's signature
//  uses a custom type (like PriorityMode or PathSource) that is
//  declared LATER in the file, the auto-generated prototype ends up
//  referencing an as-yet-undeclared type, causing errors like:
//    "'PriorityMode' was not declared in this scope"
//  Fix: define these enums here, at the very top, before anything
//  else -- so they're already in scope no matter where the IDE
//  inserts its auto-generated prototypes.
// =================================================================
enum PathSource { PATH_SOURCE_CHECKPOINT, PATH_SOURCE_MANUAL };
enum PriorityMode { PRIORITY_LEFT, PRIORITY_RIGHT, PRIORITY_FRONT, PRIORITY_NONE };

// =================================================================
//  *** GLOBALS NEEDED BY autoDetectTrackMode() DECLARED EARLY ***
//  Same issue as above, but for variables instead of types:
//  autoDetectTrackMode() (defined further down, in the "LIVE AUTO
//  MODE-DETECTION" block) reads sensorThresh[] and JUNCTION_COUNT.
//  Arduino only auto-forward-declares FUNCTIONS, not globals, so if
//  these stayed declared in their original spots (Calibration /
//  Junction sections, further down the file) the compiler would hit
//  them before they exist -> "'sensorThresh' was not declared in
//  this scope". Declaring them here fixes it.
// =================================================================
int sensorMin[8], sensorMax[8], sensorThresh[8];
int JUNCTION_COUNT = 5;   // updated by calibrate() and toggleMode()

// -- OLED -----------------------------
// 1.3" I2C OLED, 128x64, SSH1106 controller
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// -- Motor Pins -----------------------
#define ENA 3
#define IN1 4
#define IN2 2

#define ENB 11
#define IN3 7
#define IN4 6

#define STBY 13

// -- Buttons --------------------------
#define BTN_START 10
#define BTN_STOP  8

// -------------------------------------------------------------
//  MODE TOGGLE:
//    While robot is STOPPED (not running):
//      -> Short press BTN_STOP (< LONG_PRESS_MS)   : toggle NORMAL <-> INVERTED
//      -> Long  press BTN_STOP (>= LONG_PRESS_MS)   : enter PROGRAM PATH mode
//    While robot is RUNNING:
//      -> Any  press BTN_STOP              : stop the robot (unchanged)
//
//  Toggling updates INVERTED_TRACK, JUNCTION_COUNT, and the OLED.
//  The new mode persists until toggled again or power-cycled.
//
//  * AUTOMATIC POLARITY SWITCH WHILE RUNNING *
//    See the "LIVE AUTO MODE-DETECTION" block below. Every loop,
//    readSensors() checks whether the CURRENT polarity still makes
//    sense of the sensor data (a sane thin-line reading). If it
//    doesn't, but the OPPOSITE polarity does, INVERTED_TRACK flips
//    automatically (with a tiny 2-sample noise guard) -- no button
//    press needed. A real stop-bar/cross junction makes BOTH
//    polarities look invalid at once, so it never falsely triggers.
// -------------------------------------------------------------

#define LONG_PRESS_MS 600

// -- Sensors --------------------------
const byte sensorPin[8] = {A7, A6, A5, A4, A3, A2, A1, A0};

// ===============================================================
//  *  TRACK MODE  --  AUTO DETECTED then MANUALLY TOGGLEABLE  *
//     ALSO now auto-switched live while running (see below).
// ===============================================================
bool INVERTED_TRACK = false;   // auto-set in calibrate(); toggle via BTN_STOP or auto-switch

// -- PID CONFIG -----------------------
float Kp = 0.06;
float Kd = 0;

float lastError = 0;

// -- Speed ---------------------------
int BASE_SPEED   = 130;   // now selectable at startup from SPEED_OPTIONS
int MAX_SPEED    = 255;
int MOTOR_MIN    = 80;
int SEARCH_SPEED = 130;

// ===============================================================
//  *  GAP-IN-LINE HANDLING  --  TUNE HERE  *
// ===============================================================
//  Problem this solves: a physical GAP/break in the track looks
//  identical, for one instant, to a fully LOST line (all sensors
//  read '0'). Without this, the bot would immediately start
//  spinning to search, even for a tiny gap it could've just
//  driven straight across.
//
//  IMPORTANT: a real gap and a sharp turn/junction can look
//  IDENTICAL for that one instant (thin, centered line -> suddenly
//  nothing). There is no perfect way to tell them apart from a
//  single sensor frame. So instead of guessing "straight ahead",
//  the probe COASTS using the LAST commanded motor speeds
//  (lastLeftSpeed/lastRightSpeed) instead of forcing straight. If
//  the bot was already curving into a turn, it keeps curving during
//  the probe, which helps it follow into corners instead of
//  plowing straight through them. If it was tracking straight, it
//  coasts straight, which bridges a real gap.
//
//  On top of that, two safety nets stop it from ever getting stuck
//  in a "creep forward forever" loop:
//    - A reacquire must be SOLID (at least 2 sensors lit), not a
//      single noisy sensor flicker, or it doesn't count as "found".
//    - After GAP_MAX_CONSECUTIVE_ATTEMPTS failed gap-events in a
//      row, it stops trying the gap-probe at all and forces the
//      real spin-search -- guaranteeing it can never loop forever.
//
//  TUNE:
//    GAP_PROBE_COUNT   -> how many coast+check attempts per event
//                          before giving up on that event.
//    GAP_MOVE_MS       -> duration (ms) of EACH coast attempt.
//                          Bigger = bridges wider gaps, but travels
//                          further blind if it's actually a turn.
//    GAP_MOVE_SPEED    -> fallback speed only used if no previous
//                          PID speeds exist yet (very first loops).
//    GAP_MAX_ERROR_FOR_PROBE -> only allow a probe if the bot was
//                          roughly centered (small |error|, range
//                          ~0-3500) right before losing the line.
//                          Large error = it was already turning
//                          hard -> skip straight to spin-search.
//    GAP_MAX_LINE_WIDTH -> only allow a probe if the line was
//                          NARROW (few sensors lit) right before
//                          vanishing. A widening line (fork/
//                          junction spreading out) skips straight
//                          to spin-search instead.
//    GAP_MAX_CONSECUTIVE_ATTEMPTS -> hard cap on how many gap-probe
//                          events in a row are allowed to fail
//                          before forcing a real spin-search
//                          regardless of the checks above. This is
//                          what prevents endless "creep forward"
//                          loops on a real corner. Lower = commits
//                          to spin-search sooner/safer; higher =
//                          more patience for a genuinely wide gap.
// ===============================================================
int GAP_PROBE_COUNT   = 1;     // number of short coast attempts per event
unsigned long GAP_MOVE_MS = 150;    // duration of each coast attempt, in ms
int GAP_MOVE_SPEED   = 140;    // fallback speed if no prior PID speed exists yet
int GAP_MAX_ERROR_FOR_PROBE = 800;
int GAP_MAX_LINE_WIDTH = 3;
int GAP_MAX_CONSECUTIVE_ATTEMPTS = 1;   // 1 = only ever try once before forcing spin-search

// ===============================================================
//  *  LIVE AUTO MODE-DETECTION  --  no button needed  *
// ===============================================================
//  Problem this solves: some tracks change partway through from a
//  BLACK line on WHITE background to a WHITE line on BLACK
//  background (or vice-versa). readSensors() interprets raw ADC
//  values differently depending on INVERTED_TRACK, so if the
//  physical track flips polarity but INVERTED_TRACK doesn't, the
//  bot misreads every sensor from that point on.
//
//  HOW IT WORKS: every loop, readSensors() grabs raw ADC values and
//  asks autoDetectTrackMode() whether the CURRENT interpretation
//  (NORMAL or INVERTED) still yields a sane "thin line" reading
//  (between MIN_LINE_COUNT and MAX_LINE_COUNT sensors lit).
//
//  If the CURRENT mode's reading is NOT sane, but the OPPOSITE
//  mode's reading IS sane, that's a strong signal the line color
//  scheme just changed -- so we switch modes almost instantly (only
//  MODE_VOTE_THRESHOLD confirming samples, to shrug off single-frame
//  noise), rather than waiting many loops. This avoids the robot
//  "losing the line" and spinning/searching while it waits to
//  decide to switch, and reacts BEFORE reaching a fully-saturated
//  turn (which would otherwise be indistinguishable from the real
//  stop-bar in a single frame).
//
//  Cross junctions / all-black stop bars make BOTH interpretations
//  look invalid (0 or 8 sensors lit under one polarity, 8 or 0
//  under the other) -- so they never trigger a false switch.
// ===============================================================
int modeVoteCounter = 0;
const int MODE_VOTE_THRESHOLD = 2;    // just a tiny noise guard, not a slow filter
const int MIN_LINE_COUNT = 1;
const int MAX_LINE_COUNT = 5;

void autoDetectTrackMode(int raw[8]) {
  int normalCount = 0, invertedCount = 0;

  for (int i = 0; i < 8; i++) {
    if (raw[i] > sensorThresh[i]) normalCount++;   // would read '1' under NORMAL
    else                          invertedCount++;  // would read '1' under INVERTED
  }

  bool normalValid   = (normalCount   >= MIN_LINE_COUNT && normalCount   <= MAX_LINE_COUNT);
  bool invertedValid = (invertedCount >= MIN_LINE_COUNT && invertedCount <= MAX_LINE_COUNT);

  bool currentValid  = INVERTED_TRACK ? invertedValid : normalValid;
  bool oppositeValid = INVERTED_TRACK ? normalValid   : invertedValid;

  if (!currentValid && oppositeValid) {
    // Current mode makes no sense of what's under the sensors right
    // now, but the opposite mode does -> the surface has flipped.
    modeVoteCounter++;
    if (modeVoteCounter >= MODE_VOTE_THRESHOLD) {
      INVERTED_TRACK = !INVERTED_TRACK;
      JUNCTION_COUNT = INVERTED_TRACK ? 6 : 5;
      modeVoteCounter = 0;
      // No blocking delay here on purpose -- this can fire mid-run.
      // OLED will reflect it next time showReady() is drawn.
    }
  } else {
    modeVoteCounter = 0;   // stayed consistent (or genuinely ambiguous) -> reset
  }
}

// ===============================================================
//  *  BASE SPEED MENU  --  select 110/120/130/140/150/160 at startup  *
// ===============================================================
//    BTN_START (tap)         : cycle through the speed options
//    BTN_STOP  (hold >=2s)    : confirm selection, apply to BASE_SPEED
// ===============================================================
const int SPEED_OPTIONS[6] = {110, 120, 130, 140, 150, 160};
const int SPEED_OPTIONS_COUNT = 6;
int speedOptionIndex = 2;   // default -> 130

// -- Junction / branch handling ------
int JUNCTION_TURN_SPEED = 150;
int JUNCTION_HOLD_MS    = 220;

// -- Calibration ---------------------
char sensorStr[9];

// -- State ---------------------------
bool running = false;

// -- Lost-line memory ----------------
int lastDir = 1;

// -- Stop bar ------------------------
bool allBlackForward = false;

// -- Gap-vs-junction discrimination --
// Tracks the most recent sensor "width" (how many sensors were lit)
// from the last frame that saw ANY line at all. Used to tell a real
// gap (line was narrow right before vanishing) apart from a
// junction/fork (line widened before vanishing). See
// GAP_MAX_LINE_WIDTH above.
int lastLineWidth = 0;

// The most recent motor speeds PID actually commanded, so a gap
// probe can COAST that same curvature instead of forcing straight.
int lastLeftSpeed  = 0;
int lastRightSpeed = 0;

// How many gap-probe EVENTS in a row have failed to solidly
// reacquire the line. Reset to 0 the moment solid tracking resumes.
// See GAP_MAX_CONSECUTIVE_ATTEMPTS above.
int consecutiveGapAttempts = 0;

// ===============================================================
//  ***  STRING PATH  --  pre-programmed junction directions  ***
// ===============================================================
//  Example: "LLFRL" -> 1st junction = Left, 2nd = Left, 3rd = Front,
//           4th = Right, 5th = Left. After the string is used up,
//           the bot falls back to the PRIORITY logic (see below).
// ===============================================================

#define PATH_MAX_LEN 40
char pathString[PATH_MAX_LEN + 1] = "";   // saved sequence, e.g. "LLFRL"
int  pathLength   = 0;                    // how many valid letters are saved
int  pathIndex    = 0;                    // how many letters consumed this run

// ===============================================================
//  ***  PATH SOURCE  --  Checkpoints (predefined) vs Manual  ***
// ===============================================================
//    PATH_SOURCE_CHECKPOINT : pathString copied from a predefined
//                              checkpoint constant below.
//    PATH_SOURCE_MANUAL     : pathString built via the button-press
//                              editor (enterProgramPathMode()).
//    (enum PathSource itself is declared at the very top of the
//    file -- see "CUSTOM TYPES DECLARED FIRST" -- to avoid an
//    Arduino IDE 1.6.x auto-prototype ordering bug.)
// ===============================================================
PathSource pathSource = PATH_SOURCE_MANUAL;

// -- Predefined checkpoint strings (edit these as needed) --------
const char* CHECKPOINTS[8] = {
  "RLLRRLRLRRRLRLRRFRR",   // Checkpoint 1
  "RLLRRLRLRRLRLRRFRR",    // Checkpoint 2
  "RLRLRLLRL",    // Checkpoint 3
  "RLRLRLRL",    // Checkpoint 4
  "LLRL",    // Checkpoint 5
  "LRL",    // Checkpoint 6
  "RLRLRR",    // Checkpoint 7
  "RLRLR"     // Checkpoint 8
};
const int CHECKPOINT_COUNT = 8;
int checkpointIndex = 0;

// ===============================================================
//  ***  PRIORITY MODE  --  fallback turn direction after the
//        programmed path string is exhausted (or unset)  ***
// ===============================================================
//    PRIORITY_LEFT  : at every junction turn LEFT; when line is
//                      lost, rotate LEFT to search for it.
//    PRIORITY_RIGHT : mirror of the above, turning/searching RIGHT.
//    PRIORITY_FRONT : go straight through junctions; when line is
//                      lost, use previous-direction memory to search
//                      (original behavior).
//    PRIORITY_NONE  : NO special junction handling at all -- junctions
//                      are ignored completely and the bot just runs
//                      plain PID line-following the whole time
//                      (still uses direction-memory search on lost
//                      line, same as FRONT's lost-line behavior).
//    (enum PriorityMode itself is declared at the very top of the
//    file -- see "CUSTOM TYPES DECLARED FIRST" -- to avoid an
//    Arduino IDE 1.6.x auto-prototype ordering bug.)
// ===============================================================
PriorityMode priorityMode = PRIORITY_RIGHT;   // default matches old hardcoded behavior

// ========= FORWARD DECLARATIONS =========
void showReady();
void toggleMode();
void enterProgramPathMode();
void selectBaseSpeedMenu();
void selectPathSourceMenu();
void selectCheckpointMenu();
void selectPriorityMenu();
unsigned long waitReleaseAndMeasure(int pin);


// -------------------------------------------------------------
//  helper: text for priority mode
// -------------------------------------------------------------
const char* priorityName(PriorityMode p) {
  if (p == PRIORITY_LEFT) return "LEFT";
  if (p == PRIORITY_RIGHT) return "RIGHT";
  if (p == PRIORITY_FRONT) return "FRONT";
  return "NO PRIORITY";
}

// -------------------------------------------------------------
//  showReady()  -- shared helper to (re)paint the READY screen
// -------------------------------------------------------------
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

// -------------------------------------------------------------
//  toggleMode()  -- flip INVERTED_TRACK, update dependants,
//                  flash confirmation on OLED.
//  Called from loop() when BTN_STOP is short-pressed while stopped.
// -------------------------------------------------------------
void toggleMode() {
  INVERTED_TRACK = !INVERTED_TRACK;

  // Keep junction threshold in sync with the active mode
  JUNCTION_COUNT = INVERTED_TRACK ? 6 : 5;
  modeVoteCounter = 0;   // clear any pending auto-vote so it doesn't fight the manual change

  // -- Confirmation splash ----------------------------------
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("MODE TOGGLED!");
  display.println("");
  display.println(INVERTED_TRACK ? ">> INVERTED <<" : ">>  NORMAL  <<");
  display.println("");
  display.println(INVERTED_TRACK ? "(white line)" : "(black line)");
  display.display();
  delay(1500);   // brief user feedback -- only fires on manual toggle

  showReady();   // return to normal idle screen
}

// -------------------------------------------------------------
//  waitReleaseAndMeasure() -- helper: assumes pin is already LOW,
//  blocks until release, returns hold duration in ms.
// -------------------------------------------------------------
unsigned long waitReleaseAndMeasure(int pin) {
  unsigned long pressStart = millis();
  while (digitalRead(pin) == LOW) {
    delay(5);
  }
  return millis() - pressStart;
}

// -------------------------------------------------------------
//  drawSpeedMenu() -- draws the base-speed selection screen
// -------------------------------------------------------------
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

// -------------------------------------------------------------
//  selectBaseSpeedMenu() -- interactive menu using:
//    BTN_START (D10) tap  : cycle through SPEED_OPTIONS
//    BTN_STOP  (D12) hold : confirm current option -> BASE_SPEED
//  Runs once at startup (after calibration, before READY screen).
// -------------------------------------------------------------
void selectBaseSpeedMenu() {
  drawSpeedMenu(speedOptionIndex);

  while (true) {

    // -- BTN_START tap: cycle through speed options ----------
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);   // wait for release
      delay(30);   // debounce

      speedOptionIndex++;
      if (speedOptionIndex >= SPEED_OPTIONS_COUNT) speedOptionIndex = 0;

      drawSpeedMenu(speedOptionIndex);
    }

    // -- BTN_STOP hold: confirm selection --------------------
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

// -------------------------------------------------------------
//  drawPathSourceMenu() -- Checkpoints vs Manual
// -------------------------------------------------------------
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

// -------------------------------------------------------------
//  selectPathSourceMenu() -- choose between Checkpoints & Manual.
//  Runs at startup, after speed menu, before READY screen.
// -------------------------------------------------------------
void selectPathSourceMenu() {
  drawPathSourceMenu(pathSource);

  while (true) {

    // -- BTN_START tap: toggle Checkpoints <-> Manual --------
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);
      delay(30);

      pathSource = (pathSource == PATH_SOURCE_CHECKPOINT) ? PATH_SOURCE_MANUAL : PATH_SOURCE_CHECKPOINT;
      drawPathSourceMenu(pathSource);
    }

    // -- BTN_STOP hold: confirm selection --------------------
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

// -------------------------------------------------------------
//  drawCheckpointMenu() -- shows the checkpoint index + preview
// -------------------------------------------------------------
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

// -------------------------------------------------------------
//  selectCheckpointMenu() -- cycle through CHECKPOINTS[0..7] and
//  confirm one, copying it into pathString / pathLength.
// -------------------------------------------------------------
void selectCheckpointMenu() {
  drawCheckpointMenu(checkpointIndex);

  while (true) {

    // -- BTN_START tap: next checkpoint ----------------------
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);
      delay(30);

      checkpointIndex++;
      if (checkpointIndex >= CHECKPOINT_COUNT) checkpointIndex = 0;

      drawCheckpointMenu(checkpointIndex);
    }

    // -- BTN_STOP hold: confirm this checkpoint --------------
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

// -------------------------------------------------------------
//  drawPriorityMenu() -- Left / Right / Front / No Priority selection
// -------------------------------------------------------------
void drawPriorityMenu(PriorityMode sel) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("CHOOSE PRIORITY");
  display.println("");

  display.print(sel == PRIORITY_LEFT ? "> " : "  ");
  display.println("Left");

  display.print(sel == PRIORITY_RIGHT ? "> " : "  ");
  display.println("Right");

  display.print(sel == PRIORITY_FRONT ? "> " : "  ");
  display.println("Front");

  display.print(sel == PRIORITY_NONE ? "> " : "  ");
  display.println("No Priority");

  display.println("");
  display.println("TAP D10 = next");
  display.println("HOLD D12 = select");
  display.display();
}

// -------------------------------------------------------------
//  selectPriorityMenu() -- choose the fallback junction/search
//  direction used once the programmed path string runs out.
// -------------------------------------------------------------
void selectPriorityMenu() {
  drawPriorityMenu(priorityMode);

  while (true) {

    // -- BTN_START tap: cycle Left -> Right -> Front -> No Priority -> Left -
    if (digitalRead(BTN_START) == LOW) {
      while (digitalRead(BTN_START) == LOW) delay(5);
      delay(30);

      if (priorityMode == PRIORITY_LEFT) priorityMode = PRIORITY_RIGHT;
      else if (priorityMode == PRIORITY_RIGHT) priorityMode = PRIORITY_FRONT;
      else if (priorityMode == PRIORITY_FRONT) priorityMode = PRIORITY_NONE;
      else priorityMode = PRIORITY_LEFT;

      drawPriorityMenu(priorityMode);
    }

    // -- BTN_STOP hold: confirm selection --------------------
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

// -------------------------------------------------------------
//  drawPathEditor() -- draws the live edit screen:
//  shows the string built so far, the cursor slot, and the
//  letter currently being cycled for that slot.
// -------------------------------------------------------------
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

// -------------------------------------------------------------
//  enterProgramPathMode() -- interactive editor using only
//  BTN_START (tap = cycle letter) and BTN_STOP (hold = confirm
//  slot & advance). Holding BTN_STOP on a BLANK slot ends and
//  saves the string. Used when pathSource == PATH_SOURCE_MANUAL.
// -------------------------------------------------------------
void enterProgramPathMode() {
  int slot = 0;
  char options[3] = {'L', 'F', 'R'};
  int optIndex = -1;          // -1 means "blank" (no letter chosen yet)
  char currentLetter = 0;     // 0 = blank

  drawPathEditor(currentLetter, slot);

  while (true) {

    // -- BTN_START tap: cycle L -> F -> R -> blank -> L ... --
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

    // -- BTN_STOP hold: confirm current slot & advance -------
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

// ========= MOTOR =========

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

// ========= SENSOR =========

void readSensors() {
  int raw[8];
  for (int i = 0; i < 8; i++) {
    raw[i] = analogRead(sensorPin[i]);
  }

  // * Live auto mode-detection -- may flip INVERTED_TRACK right here *
  autoDetectTrackMode(raw);

  for (int i = 0; i < 8; i++) {
    if (INVERTED_TRACK) {
      // White line on black surface:
      // white = high reflectance = LOW  ADC value  -> '1' (line)
      // black = low  reflectance = HIGH ADC value  -> '0' (background)
      sensorStr[i] = (raw[i] < sensorThresh[i]) ? '1' : '0';
    } else {
      // Black line on white surface:
      // black = low  reflectance = HIGH ADC value  -> '1' (line)
      // white = high reflectance = LOW  ADC value  -> '0' (background)
      sensorStr[i] = (raw[i] > sensorThresh[i]) ? '1' : '0';
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

// -------------------------------------------------------------
//  allBlack() -- name kept for compatibility.
//  Checks: "are ALL sensors reading the LINE?"
//  Works identically for both NORMAL and INVERTED modes.
// -------------------------------------------------------------
bool allBlack() {
  for (int i = 0; i < 8; i++) {
    if (sensorStr[i] == '0') return false;
  }
  return true;
}

// ========= IMPROVED DETECTION =========

// [OK] Real junction -- uses JUNCTION_COUNT (tunable per track mode)
// ========= IMPROVED JUNCTION DETECTION (shape-based) =========

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

// [OK] Junction = any recognized shape OR the old generic threshold as fallback
bool isJunction() {
  if (isCross() || isLeftT() || isRightT()) return true;

  int count = 0;
  for (int i = 0; i < 8; i++)
    if (sensorStr[i] == '1') count++;
  return (count >= JUNCTION_COUNT);
}

// [OK] Strong center detection
bool centerActive() {
  return (sensorStr[3] == '1' && sensorStr[4] == '1');
}

// [OK] Clean branch detection (ignore noise)
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

// ========= CALIBRATION =========

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

  // * AUTO DETECT INVERTED MODE *
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

  // -- Serial verify -----------------------------------------
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

// ========= SETUP =========

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

  // * select base speed (110/120/130/140/150/160) *
  selectBaseSpeedMenu();

  // * NEW: choose path source -- Checkpoints or Manual *
  selectPathSourceMenu();

  if (pathSource == PATH_SOURCE_CHECKPOINT) {
    selectCheckpointMenu();     // pick one of the predefined strings
  } else {
    enterProgramPathMode();     // build the string by hand, as before
  }

  // * NEW: choose fallback priority -- Left / Right / Front / No Priority *
  selectPriorityMenu();

  showReady();   // paint READY + mode + speed + path + priority
}

// ========= LOOP =========

void loop() {

  // -- START button ------------------------------------------
  if (digitalRead(BTN_START) == LOW) {
    running = true;
    pathIndex = 0;          // reset string-path progress for the new run
    delay(200);
  }

  // -- STOP button -------------------------------------------
  if (digitalRead(BTN_STOP) == LOW) {

    if (running) {
      // -- Robot is running -> STOP immediately ---------------
      running = false;
      stopMotors();
      delay(200);   // debounce
      showReady();

    } else {
      // -- Robot is already stopped -> measure press length ---
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
  // Note: live auto mode-detection (autoDetectTrackMode) already ran
  // inside readSensors() above, using fresh raw ADC values -- see the
  // "LIVE AUTO MODE-DETECTION" block near the top of the file. If the
  // track's color scheme changed, INVERTED_TRACK is already corrected
  // and sensorStr above reflects it, so no extra re-read is needed here.

  // -- Track line width every frame that sees ANY line, so the
  //    gap-vs-junction check below knows what the line looked like
  //    right before it possibly disappears.
  {
    int w = 0;
    for (int i = 0; i < 8; i++) if (sensorStr[i] == '1') w++;
    if (w > 0) lastLineWidth = w;
  }

  // [OK] STOP BAR
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

  // ===========================================================
  //  STRING PATH ALWAYS RUNS FIRST -- regardless of priority mode
  //  (including NO PRIORITY). Only once pathIndex reaches
  //  pathLength does control pass to the selected priority logic.
  // ===========================================================

  // [BRANCH] FULL JUNCTION
  if (isJunction()) {

    // === STRING PATH OVERRIDE -- always takes priority ===
    // If we still have un-consumed letters in the programmed
    // path, use that direction instead of anything else,
    // no matter which priority mode is selected.
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
      } else { // 'F' -- go straight through the junction
        motorLeft(BASE_SPEED, true);
        motorRight(BASE_SPEED, true);
        delay(JUNCTION_HOLD_MS);
      }
      return;
    }

    // -- Path string exhausted (or empty) -- hand off to PRIORITY mode --

    // NO PRIORITY: ignore this junction entirely, just keep line-following.
    if (priorityMode == PRIORITY_NONE) {
      // fall through to PID below -- no special junction action
    } else if (priorityMode == PRIORITY_LEFT) {
      motorLeft(JUNCTION_TURN_SPEED, false);
      motorRight(JUNCTION_TURN_SPEED, true);
      delay(JUNCTION_HOLD_MS);
      lastDir = -1;
      return;
    } else if (priorityMode == PRIORITY_RIGHT) {
      motorLeft(JUNCTION_TURN_SPEED, true);
      motorRight(JUNCTION_TURN_SPEED, false);
      delay(JUNCTION_HOLD_MS);
      lastDir = 1;
      return;
    } else { // PRIORITY_FRONT
      motorLeft(BASE_SPEED, true);
      motorRight(BASE_SPEED, true);
      delay(JUNCTION_HOLD_MS);
      return;
    }
  }

  // [BRANCH] CENTER + SIDE BRANCH
  // Still respect an unfinished path string: while the path is
  // active we don't want the generic side-branch turner fighting
  // with pathIndex, so only run this once the path is exhausted
  // AND we're not in NO PRIORITY mode.
  if (pathIndex >= pathLength && priorityMode != PRIORITY_NONE && centerActive()) {
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

  if (pos != -1) {
    // Solid tracking this frame -- line drama (if any) is over.
    consecutiveGapAttempts = 0;
  }

  // [OK] GAP-IN-LINE CHECK, then STRONG LOST LINE RECOVERY
  if (pos == -1) {

    // Only attempt a gap-probe if:
    //   - the line was NARROW right before vanishing (not a
    //     widening fork/junction), AND
    //   - the bot was roughly centered (not already mid-turn), AND
    //   - we haven't already burned through our consecutive-attempt
    //     budget on this same lost-line episode (this is what
    //     guarantees we can never loop "creep forward" forever).
    bool looksLikeGap =
      (lastLineWidth <= GAP_MAX_LINE_WIDTH) &&
      (fabs(lastError) <= GAP_MAX_ERROR_FOR_PROBE) &&
      (consecutiveGapAttempts < GAP_MAX_CONSECUTIVE_ATTEMPTS);

    if (looksLikeGap) {
      consecutiveGapAttempts++;

      // Coast using the LAST commanded PID speeds instead of forcing
      // straight. If the bot was already curving into a turn, this
      // keeps curving through the probe instead of blindly going
      // straight. If it was tracking straight, it coasts straight,
      // which is exactly right for bridging a real gap.
      int coastLeft  = lastLeftSpeed;
      int coastRight = lastRightSpeed;
      if (coastLeft == 0 && coastRight == 0) {
        // No prior PID speed yet (e.g. very first loops) -- fall back
        // to a plain straight creep.
        coastLeft = GAP_MOVE_SPEED;
        coastRight = GAP_MOVE_SPEED;
      }

      for (int probe = 0; probe < GAP_PROBE_COUNT; probe++) {
        motorLeft(coastLeft, true);
        motorRight(coastRight, true);
        delay(GAP_MOVE_MS);

        readSensors();
        pos = getPosition();

        int w = 0;
        for (int i = 0; i < 8; i++) if (sensorStr[i] == '1') w++;

        if (pos != -1 && w >= 2) {
          // SOLID reacquire (not just single-sensor noise) -- it was
          // just a gap. Reset the attempt counter and let the
          // normal PID block below handle it with fresh data.
          consecutiveGapAttempts = 0;
          break;
        } else {
          // Noisy/weak or still nothing -- treat as still lost and
          // keep the loop (or fall through to spin-search after).
          pos = -1;
        }
      }
    }
  }

  // NOTE: the auto-polarity-switch check now runs once per frame,
  // right after readSensors() near the top of loop() -- not here.
  // That's necessary because rolling onto an inverted section
  // reads as a SATURATED sensor bar (looks like a junction/cross),
  // not as a lost line (pos == -1), so it has to be checked before
  // junction detection even happens, not only in this lost-line
  // branch. This block still handles the genuine "nothing lit at
  // all under either polarity" case via the normal spin-search.

  if (pos == -1) {
    // Still nothing solid after gap handling AND polarity check --
    // genuinely lost the line (or a turn/junction we correctly
    // declined to creep through). Original search/spin behavior,
    // unchanged.

    if (priorityMode == PRIORITY_LEFT) {
      // Always rotate left to search, per LEFT priority
      motorLeft(SEARCH_SPEED, false);
      motorRight(SEARCH_SPEED, true);
    } else if (priorityMode == PRIORITY_RIGHT) {
      // Always rotate right to search, per RIGHT priority
      motorLeft(SEARCH_SPEED, true);
      motorRight(SEARCH_SPEED, false);
    } else {
      // PRIORITY_FRONT and PRIORITY_NONE: fall back to
      // previous-direction memory (original behavior)
      if (lastDir > 0) {
        motorLeft(SEARCH_SPEED, true);
        motorRight(SEARCH_SPEED, false);
      } else {
        motorLeft(SEARCH_SPEED, false);
        motorRight(SEARCH_SPEED, true);
      }
    }

    return;
  }

  // [PID] PID
  float error = pos - 3500;

  // [OK] Direction memory -- only update on significant deviation
  if (error > 600) lastDir = 1;
  else if (error < -600) lastDir = -1;

  float derivative = error - lastError;
  float correction  = Kp * error + Kd * derivative;
  lastError = error;

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  lastLeftSpeed  = leftSpeed;
  lastRightSpeed = rightSpeed;

  motorLeft(leftSpeed,  true);
  motorRight(rightSpeed, true);
}
