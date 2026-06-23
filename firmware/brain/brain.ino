/*
 * BRAIN — XYZ gantry pick & place with QR sorting  (the real firmware)
 * ===================================================================
 * Board: ESP32-DevKitC WROOM-32D ("the brain", CLAUDE.md §1), 3x A4988 on a
 * CNC Shield V3. This is the consolidated brain code; the test sketches
 * (brain_x_test, brain_xyz_oscillate, brain_xyz_limit_test) stay as fallbacks.
 *
 * Built in stages (see CLAUDE.md §9):
 *   [A] motion + soft limits + endstops + calibration + HOMING (Z->X->Y)  <-- THIS FILE
 *   [B] SG90 gripper + teach positions (PICK/SCAN/SHELF)
 *   [C] UART2 link to ESP32-CAM + QR parsing
 *   [D] full pick->scan->decide->place state machine + reject bin
 *
 * Decisions locked in: single shared steps_per_mm; positions hardcoded in
 * this file (no NVS) — teach by jogging, read the printed steps, paste below.
 *
 * ─── Pin map (CLAUDE.md §4.2) ───────────────────────────────────────────────
 *   X STEP25 DIR26   Y STEP32 DIR33   Z STEP27 DIR14   EN(all)13 active-LOW
 *   MIN/home  X21 Y22 Z23      MAX/limit  X4 Y18 Z17
 *   Servo19 (stage B)   UART2 RX16 / TX5 -> CAM (stage C)
 *
 * ─── Serial console (115200, newline) ───────────────────────────────────────
 *   H[axis]   home: HA=all (Z->X->Y), or HX/HY/HZ one axis
 *   J<axis><mm>  jog relative (JX50 / JZ-10)      [endstop + soft-limit safe]
 *   G<axis><mm>  goto absolute mm (GX120)         [endstop + soft-limit safe]
 *   C<axis><mm>  calibrate: jog known dist, then type the MEASURED mm
 *   V<mm/s>  speed     A<mm/s2> accel     S stop     P status     ?  help
 */

#include "FastAccelStepper.h"
#include <WiFi.h>
#include <esp_now.h>

// ── Endstop convention ──────────────────────────────────────────────────────
// Current hardware: NO (Normally Open) switches, COM->GND, INPUT_PULLUP.
//   at rest = open -> HIGH = NOT tripped ;  pressed = GND -> LOW = TRIPPED.
// (For the final robot, NC + this set to HIGH is fail-safe — see CLAUDE.md §5.3.)
#define ENDSTOP_TRIPPED_LEVEL LOW
#define DEBOUNCE_SAMPLES 4

// ── Axes ────────────────────────────────────────────────────────────────────
struct Axis {
  const char *name;
  uint8_t step, dir;
  uint8_t minPin, maxPin;          // MIN = home end, MAX = far end
  FastAccelStepper *s;
};
enum { AX_X = 0, AX_Y = 1, AX_Z = 2 };
Axis AXES[] = {
  { "X", 25, 26, 21,  4, nullptr },
  { "Y", 32, 33, 22, 18, nullptr },
  { "Z", 27, 14, 23, 17, nullptr },
};
const int N_AXES = sizeof(AXES) / sizeof(AXES[0]);
#define ENABLE_PIN 13              // shared A4988 enable, ACTIVE LOW

// A taught XYZ position in mm (Z is negative-down, 0 = top/safe). Defined up here
// so the .ino auto-prototype generator sees the type before any function signature.
struct Pos { float x, y, z; };

// ── ESP32-CAM link (UART2, §8) ──────────────────────────────────────────────
//   CAM TX(GPIO13) -> brain RX(GPIO16) : "QR:<code>\n"
//   brain TX(GPIO5) -> CAM RX(GPIO14)  : "SCAN\n" (only used in trigger mode)
//   Common GND. Both 3.3V -> direct, no level shifter.
#define CAM_RX_PIN 16      // (UART link unused now — camera comes in over ESP-NOW)
#define CAM_TX_PIN 5
HardwareSerial Cam(2);             // Serial2 to the ESP32-CAM

// ── Calibration & motion (shared steps_per_mm, per decision) ────────────────
float steps_per_mm = 50.0f;        // FULL STEP, 4mm lead. SET after C-calibration!
float speed_mm_s   = 20.0f;
float accel_mm_s2  = 600.0f;

// Soft limits per axis [min, max] in mm (after homing). 0,0 = disabled (only the
// hardware endstops protect that axis). Mind the coordinate sense per axis:
//   X,Y home at MIN -> 0 at home, travel is POSITIVE   -> set max > 0
//   Z   homes at MAX -> 0 at the TOP, DOWN is NEGATIVE  -> set min < 0 (e.g. -120), max = 0
//                              X      Y       Z      (measured travel: X197.82 Y168.28 Z-120, minus ~3mm margin)
float soft_min_mm[N_AXES] = {    0,     0,   -117 };   // {X, Y, Z}
float soft_max_mm[N_AXES] = {  194,   170,      0 };   // {X, Y, Z}

// Homing parameters
float HOME_FAST_MM_S = 15.0f;      // fast approach
float HOME_SLOW_MM_S = 3.0f;       // slow re-approach (removes bounce error)
float HOME_BACKOFF_MM = 4.0f;      // pull-off between the two approaches
// Homing is configured by TWO independent per-axis knobs:
//   HOME_USES_MAX[i] : which physical switch is the home reference
//                      (true = MAX pin, false = MIN pin)
//   HOME_DIR[i]      : which way the MOTOR turns to reach it
//                      (+1 = runForward, -1 = runBackward)
// Set HOME_USES_MAX to the switch that sits at the home end. Then if the motor
// drives AWAY from that switch during homing, just flip the sign of HOME_DIR[i].
// Z homes UP (away from the shelves in front); X,Y home toward their MIN end.
const bool   HOME_USES_MAX[N_AXES] = { false, false, true };  // X->MIN, Y->MIN, Z->MAX
const int8_t HOME_DIR[N_AXES]      = { -1,    -1,    +1   };   // X->MIN, Y->MIN, Z->MAX (Z: +1 = AWAY from motor = toward MAX)
const uint32_t HOME_TIMEOUT_MS = 30000;

bool homed[N_AXES] = { false, false, false };
bool autosort = false;             // when true, a QR from the camera auto-runs a sort cycle

// ESP-NOW receive buffer (filled in the radio callback, processed in loop()).
volatile bool camMsgReady = false;
char camMsgBuf[64];

// calibration state
float lastCalMm = 0.0f;
bool  awaitingCalMeasure = false;

FastAccelStepperEngine engine = FastAccelStepperEngine();

// ── helpers ─────────────────────────────────────────────────────────────────
long  mmToSteps(float mm) { return (long)lroundf(mm * steps_per_mm); }
float stepsToMm(long st)  { return (float)st / steps_per_mm; }

int axisIndex(char c) {
  c = toupper(c);
  for (int i = 0; i < N_AXES; i++) if (AXES[i].name[0] == c) return i;
  return -1;
}

bool endstopTripped(uint8_t pin) { return digitalRead(pin) == ENDSTOP_TRIPPED_LEVEL; }

void applyMotion() {
  for (int i = 0; i < N_AXES; i++) {
    if (!AXES[i].s) continue;
    AXES[i].s->setSpeedInHz((uint32_t)lroundf(speed_mm_s * steps_per_mm));
    AXES[i].s->setAcceleration((uint32_t)lroundf(accel_mm_s2 * steps_per_mm));
  }
}

void stopAll() {
  awaitingCalMeasure = false;
  for (int i = 0; i < N_AXES; i++) if (AXES[i].s) AXES[i].s->forceStop();
}

// ── Blocking single-axis motion with endstop safety ─────────────────────────
// Runs an axis continuously in motor direction `dir` (+1 fwd / -1 back) until
// `watchPin` trips (debounced) or timeout. Returns true if the switch was hit.
bool runUntilEndstop(int i, int8_t dir, uint8_t watchPin, float speed_mm_per_s, uint32_t timeoutMs) {
  Axis &a = AXES[i];
  a.s->setSpeedInHz((uint32_t)lroundf(speed_mm_per_s * steps_per_mm));
  if (dir > 0) a.s->runForward(); else a.s->runBackward();

  uint32_t t0 = millis();
  uint8_t trip = 0;
  while (true) {
    if (endstopTripped(watchPin)) {
      if (++trip >= DEBOUNCE_SAMPLES) { a.s->forceStop(); return true; }
    } else {
      trip = 0;
    }
    if (millis() - t0 > timeoutMs) { a.s->forceStop(); return false; }
    delay(1);
  }
}

// Relative blocking move (no endstop monitoring — use for short safe back-offs).
void moveRelBlocking(int i, float mm, float speed_mm_per_s) {
  Axis &a = AXES[i];
  a.s->setSpeedInHz((uint32_t)lroundf(speed_mm_per_s * steps_per_mm));
  a.s->move(mmToSteps(mm));
  while (a.s->isRunning()) delay(1);
}

// Absolute blocking move to mm, clamped to soft limits, stops if the endstop in
// the travel direction trips on the way. Returns true if it reached the target,
// false if a limit switch cut the move short (-> e.g. calibration is invalid).
bool gotoBlockingMM(int i, float target_mm) {
  Axis &a = AXES[i];
  if (soft_min_mm[i] != 0 || soft_max_mm[i] != 0)
    target_mm = constrain(target_mm, soft_min_mm[i], soft_max_mm[i]);
  long target = mmToSteps(target_mm);
  int8_t dir = (target >= a.s->getCurrentPosition()) ? +1 : -1;
  // For all axes here, increasing position = runForward = physically toward MAX,
  // decreasing = toward MIN. So the switch "ahead" follows dir directly.
  uint8_t ahead = (dir > 0) ? a.maxPin : a.minPin;

  a.s->moveTo(target);
  uint8_t trip = 0;
  while (a.s->isRunning()) {
    if (endstopTripped(ahead)) {
      if (++trip >= DEBOUNCE_SAMPLES) {
        a.s->forceStop();
        Serial.printf("[!] %s endstop hit during move -> stopped at %.2f mm\n",
                      a.name, stepsToMm(a.s->getCurrentPosition()));
        return false;
      }
    } else trip = 0;
    delay(1);
  }
  return true;
}

// ── Homing ──────────────────────────────────────────────────────────────────
bool homeAxis(int i) {
  Axis &a = AXES[i];
  int8_t dir = HOME_DIR[i];
  uint8_t homePin = HOME_USES_MAX[i] ? a.maxPin : a.minPin;   // which switch is "home"
  const char *end = HOME_USES_MAX[i] ? "MAX" : "MIN";
  Serial.printf("[home] %s (switch %s, motor dir %+d) ...\n", a.name, end, dir);

  // if already sitting on the home switch, step off it first
  if (endstopTripped(homePin)) moveRelBlocking(i, -dir * HOME_BACKOFF_MM, HOME_SLOW_MM_S);

  // 1) fast approach toward the home switch
  if (!runUntilEndstop(i, dir, homePin, HOME_FAST_MM_S, HOME_TIMEOUT_MS)) {
    Serial.printf("[home] %s FAILED (no %s switch within timeout -> flip HOME_DIR[%s])\n",
                  a.name, end, a.name);
    return false;
  }
  // 2) back off the switch
  moveRelBlocking(i, -dir * HOME_BACKOFF_MM, HOME_SLOW_MM_S);
  // 3) slow re-approach
  if (!runUntilEndstop(i, dir, homePin, HOME_SLOW_MM_S, HOME_TIMEOUT_MS)) {
    Serial.printf("[home] %s FAILED on slow re-approach\n", a.name);
    return false;
  }
  // 4) set origin
  a.s->forceStop();
  a.s->setCurrentPosition(0);
  homed[i] = true;
  applyMotion();                     // restore user speed/accel
  Serial.printf("[home] %s OK -> 0\n", a.name);
  return true;
}

// Full homing in the anti-collision order Z -> X -> Y (CLAUDE.md §5.3).
bool homeAll() {
  Serial.println(F("[home] ALL (Z -> X -> Y)"));
  if (!homeAxis(AX_Z)) return false;
  if (!homeAxis(AX_X)) return false;
  if (!homeAxis(AX_Y)) return false;
  Serial.println(F("[home] ALL done"));
  return true;
}

// ── Gripper (SG90 servo on LEDC, GPIO19) ────────────────────────────────────
// Wiring: SG90 signal -> GPIO19, SG90 V+ -> 5V buck (NOT the ESP32 3V3!), GND common.
// TUNE grip_open_deg / grip_close_deg with the E<angle> command, then paste here.
#define SERVO_PIN  19
#define SERVO_CH   4          // LEDC channel (steppers use RMT, so this is free)
#define SERVO_FREQ 50         // 50 Hz = 20 ms period
#define SERVO_RES  16         // 16-bit duty

int grip_open_deg  = 3;       // jaws open / release the box
int grip_close_deg = 47;      // jaws closed / grip the box (50 stalls/buzzes)

uint32_t angleToDuty(int deg) {
  deg = constrain(deg, 0, 180);
  float us = 500.0f + (deg / 180.0f) * 1900.0f;          // SG90 ~0.5..2.4 ms
  return (uint32_t)lroundf(us / 20000.0f * ((1UL << SERVO_RES) - 1));
}
void servoWrite(int deg) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, angleToDuty(deg));     // core 3.x: ledcWrite takes the PIN
#else
  ledcWrite(SERVO_CH, angleToDuty(deg));      // core 2.x: ledcWrite takes the CHANNEL
#endif
}
void gripOpen()  { servoWrite(grip_open_deg);  Serial.println(F("[grip] OPEN")); }
void gripClose() { servoWrite(grip_close_deg); Serial.println(F("[grip] CLOSED")); }

// ── Shelf grid 3 rows x 2 cols + code routing (per "Box Structure.txt") ──────
//   Row1:  MS0012  MS0011
//   Row2:  CJ0011  CJ0012
//   Row3:  EB0011  [BOXES = pickup/feed zone]
// Positions hardcoded; teach with T11..T32 / TS, dump with D, paste back here.
Pos SHELF[3][2] = {                     // [row][col], mm. Z negative-down (0=top).
  { {174.94, 9.00, -110.54}, {6.92, 9.00, -110.54} },      // Row1: MS0012  MS0011
  { {164.94, 109.00, -110.36}, {6.92, 104.00, -110.54} },  // Row2: CJ0011  CJ0012
  { {170.00, 164.94, -110.54}, {-0.06, 164.94, -110.54} }, // Row3: EB0011  BOXES
};
Pos SCAN_POS = { -0.06, 164.94, -19.94 };   // where the camera reads the QR

#define PICK_ROW 2
#define PICK_COL 1                      // Row3 Col2 = "Boxes" feed zone

// Safe travel height for Z. NOT 0: Z=0 sits exactly on the MAX/home switch, so
// going there trips it. A few mm below clears the switch but still clears shelves.
#define Z_SAFE_MM -5.0f

// Exact QR code -> destination cell (fixed mapping, CLAUDE.md §6.2 strategy A).
struct CodeCell { const char *code; uint8_t row, col; };
const CodeCell CODE_MAP[] = {
  { "MS0012", 0, 0 },   // Row1 Col1
  { "MS0011", 0, 1 },   // Row1 Col2
  { "CJ0011", 1, 0 },   // Row2 Col1
  { "CJ0012", 1, 1 },   // Row2 Col2
  { "EB0011", 2, 0 },   // Row3 Col1
};
const int N_CODES = sizeof(CODE_MAP) / sizeof(CODE_MAP[0]);

bool allHomed() { return homed[0] && homed[1] && homed[2]; }

Pos curPos() {
  return { stepsToMm(AXES[AX_X].s->getCurrentPosition()),
           stepsToMm(AXES[AX_Y].s->getCurrentPosition()),
           stepsToMm(AXES[AX_Z].s->getCurrentPosition()) };
}

// Resolve a teach slot token ("S","11".."32") to a Pos* and fill its name.
Pos *resolveSlot(const String &a, char *nameOut) {
  if (a.length() == 0) return nullptr;
  if (toupper(a.charAt(0)) == 'S') { strcpy(nameOut, "SCAN"); return &SCAN_POS; }
  if (a.length() >= 2 && isDigit(a.charAt(0)) && isDigit(a.charAt(1))) {
    int r = a.charAt(0) - '1', c = a.charAt(1) - '1';
    if (r >= 0 && r < 3 && c >= 0 && c < 2) {
      sprintf(nameOut, "SHELF[%d][%d]%s", r, c, (r == PICK_ROW && c == PICK_COL) ? "=BOXES" : "");
      return &SHELF[r][c];
    }
  }
  return nullptr;
}

// Scanned QR code -> destination cell position (nullptr if the code is unknown).
Pos *routeForCode(const char *code) {
  for (int i = 0; i < N_CODES; i++)
    if (strcasecmp(code, CODE_MAP[i].code) == 0) return &SHELF[CODE_MAP[i].row][CODE_MAP[i].col];
  return nullptr;
}

// Safety: before any X/Y move, lift Z to the top (0) so the gripper clears the
// shelves (CLAUDE.md §7). Only acts when homed and Z is currently below the top.
void raiseZBeforeXY(int ai) {
  if (ai != AX_X && ai != AX_Y) return;
  if (!allHomed()) return;
  if (stepsToMm(AXES[AX_Z].s->getCurrentPosition()) < Z_SAFE_MM - 0.1f) {
    Serial.println(F("[safe] Z -> safe top before XY move"));
    gotoBlockingMM(AX_Z, Z_SAFE_MM);
  }
}

// Safe move to a position: raise Z to safe top, move XY, then lower Z (§7).
void gotoPos(const Pos &p) {
  gotoBlockingMM(AX_Z, Z_SAFE_MM);  // raise to safe height FIRST (no shelf collisions)
  gotoBlockingMM(AX_X, p.x);
  gotoBlockingMM(AX_Y, p.y);
  gotoBlockingMM(AX_Z, p.z);        // then lower onto the target
}

// Pick: open, descend onto the box, grip, lift to safe top.
void doPick(const Pos &p) { gripOpen(); gotoPos(p); gripClose(); delay(400); gotoBlockingMM(AX_Z, Z_SAFE_MM); }
// Place: descend onto the cell, release, lift to safe top.
void doPlace(const Pos &p) { gotoPos(p); gripOpen(); delay(400); gotoBlockingMM(AX_Z, Z_SAFE_MM); }

// One full sort cycle for a known code: pick from BOXES -> place at its cell.
// (Camera/scan arrives in stage C; here the code is provided directly.)
bool sortCycle(const char *code) {
  Pos *dest = routeForCode(code);
  if (!dest) { Serial.printf("[sort] unknown code '%s' -> reject (no cell)\n", code); return false; }
  Serial.printf("[sort] pick BOXES -> place %s\n", code);
  doPick(SHELF[PICK_ROW][PICK_COL]);
  doPlace(*dest);
  Serial.println(F("[sort] done"));
  return true;
}

// Print all taught positions as C code, ready to paste back into this file.
void dumpPositions() {
  Serial.println(F("---- paste into brain.ino ----"));
  Serial.printf("Pos SCAN_POS = { %.2f, %.2f, %.2f };\n", SCAN_POS.x, SCAN_POS.y, SCAN_POS.z);
  Serial.println(F("Pos SHELF[3][2] = {"));
  for (int r = 0; r < 3; r++) {
    Serial.print(F("  {"));
    for (int c = 0; c < 2; c++)
      Serial.printf(" {%.2f,%.2f,%.2f}%s", SHELF[r][c].x, SHELF[r][c].y, SHELF[r][c].z, c < 1 ? "," : "");
    Serial.println(F(" },"));
  }
  Serial.println(F("};\n------------------------------"));
}

// ── Console ─────────────────────────────────────────────────────────────────
void printStatus() {
  Serial.print(F("[P] "));
  for (int i = 0; i < N_AXES; i++) {
    Axis &a = AXES[i];
    Serial.printf("%s=%.2fmm[%c%c%c] ", a.name, stepsToMm(a.s->getCurrentPosition()),
                  homed[i] ? 'H' : '-',
                  endstopTripped(a.minPin) ? 'm' : '-',
                  endstopTripped(a.maxPin) ? 'M' : '-');
  }
  Serial.printf("| V=%.1f A=%.1f | steps/mm=%.3f\n", speed_mm_s, accel_mm_s2, steps_per_mm);
}

void printHelp() {
  Serial.println(F(
    "BRAIN stage B — motion + homing + gripper\n"
    "  H[axis]  home: HA=all(Z->X->Y) / HX / HY / HZ\n"
    "  J<axis><mm>  jog relative  (JX50 / JZ-10)\n"
    "  G<axis><mm>  goto absolute (GX120)\n"
    "  C<axis><mm>  calibrate: jog known dist, then type the MEASURED mm\n"
    "  O open grip   L close grip   E<deg> set servo angle (tune open/close)\n"
    "  T<slot> teach here   M<slot> move there   D dump  (slot: S 11..32; 32=Boxes)\n"
    "  B<code> run one sort cycle: pick Boxes -> place by code (e.g. BMS0012)\n"
    "  Q1/Q0  auto-sort on QR from the camera (on/off)\n"
    "  V<mm/s> speed   A<mm/s2> accel   S stop   P status   ?  help\n"
    "  P legend: H=homed  m=MIN tripped  M=MAX tripped"));
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = toupper(s.charAt(0));
  String arg = s.substring(1); arg.trim();

  switch (c) {
    case 'H': {
      if (arg.length() == 0 || toupper(arg.charAt(0)) == 'A') { homeAll(); break; }
      int ai = axisIndex(arg.charAt(0));
      if (ai < 0) { Serial.println(F("[H] usage: HA / HX / HY / HZ")); break; }
      homeAxis(ai);
      break;
    }
    case 'J': {
      int ai = axisIndex(arg.charAt(0));
      if (ai < 0) { Serial.println(F("[J] usage: JX50 / JZ-10")); break; }
      float mm = arg.substring(1).toFloat();
      raiseZBeforeXY(ai);
      gotoBlockingMM(ai, stepsToMm(AXES[ai].s->getCurrentPosition()) + mm);
      Serial.printf("[J] %s now %.2f mm\n", AXES[ai].name, stepsToMm(AXES[ai].s->getCurrentPosition()));
      break;
    }
    case 'G': {
      int ai = axisIndex(arg.charAt(0));
      if (ai < 0) { Serial.println(F("[G] usage: GX120")); break; }
      raiseZBeforeXY(ai);
      gotoBlockingMM(ai, arg.substring(1).toFloat());
      Serial.printf("[G] %s now %.2f mm\n", AXES[ai].name, stepsToMm(AXES[ai].s->getCurrentPosition()));
      break;
    }
    case 'C': {
      int ai = axisIndex(arg.charAt(0));
      float mm = arg.substring(1).toFloat();
      if (ai < 0 || mm == 0) { Serial.println(F("[C] usage: CX100, then type the measured mm")); break; }
      lastCalMm = fabs(mm);
      if (!gotoBlockingMM(ai, stepsToMm(AXES[ai].s->getCurrentPosition()) + mm)) {
        Serial.println(F("[C] hit a limit before finishing -> measurement INVALID."));
        Serial.println(F("    Jog away from the limit (e.g. JX-50) to mid-travel and retry."));
        break;
      }
      awaitingCalMeasure = true;
      Serial.println(F("[C] measure the real travel with calipers, then type just the number (e.g. 98.7)"));
      break;
    }
    case 'O':
      gripOpen();
      break;
    case 'L':
      gripClose();
      break;
    case 'T': {   // teach: store current position into a slot
      char name[24];
      Pos *slot = resolveSlot(arg, name);
      if (!slot) { Serial.println(F("[T] usage: TS / T11..T32 (T32 = Boxes pickup)")); break; }
      if (!allHomed()) { Serial.println(F("[T] home first (HA) so positions are referenced")); break; }
      *slot = curPos();
      Serial.printf("[T] %s = {%.2f, %.2f, %.2f}  (use D to dump all for pasting)\n",
                    name, slot->x, slot->y, slot->z);
      break;
    }
    case 'M': {   // move to a taught slot (safe Z motion)
      char name[24];
      Pos *slot = resolveSlot(arg, name);
      if (!slot) { Serial.println(F("[M] usage: MS / M11..M32")); break; }
      if (!allHomed()) { Serial.println(F("[M] home first (HA)")); break; }
      Serial.printf("[M] -> %s {%.2f, %.2f, %.2f}\n", name, slot->x, slot->y, slot->z);
      gotoPos(*slot);
      break;
    }
    case 'D':
      dumpPositions();
      break;
    case 'B': {   // run one sort cycle for a manually-entered code (camera = stage C)
      if (!allHomed()) { Serial.println(F("[B] home first (HA)")); break; }
      if (arg.length() == 0) { Serial.println(F("[B] usage: B<code>  e.g. BMS0012")); break; }
      sortCycle(arg.c_str());
      break;
    }
    case 'E': {
      int deg = arg.toInt();
      servoWrite(deg);
      Serial.printf("[E] servo -> %d deg (tune, then set grip_open_deg/grip_close_deg)\n", deg);
      break;
    }
    case 'V':
      speed_mm_s = max(0.1f, arg.toFloat()); applyMotion();
      Serial.printf("[V] speed = %.2f mm/s\n", speed_mm_s);
      break;
    case 'A':
      accel_mm_s2 = max(1.0f, arg.toFloat()); applyMotion();
      Serial.printf("[A] accel = %.2f mm/s^2\n", accel_mm_s2);
      break;
    case 'S':
      stopAll();
      Serial.println(F("[S] stopped"));
      break;
    case 'Q':
      autosort = (arg.toInt() == 1);
      Serial.printf("[Q] auto-sort on incoming QR = %s\n", autosort ? "ON" : "OFF");
      break;
    case 'P':
      printStatus();
      break;
    case '?':
      printHelp();
      break;
    default:
      if (awaitingCalMeasure) {
        float measured = s.toFloat();
        if (measured > 0) {
          float corrected = steps_per_mm * (lastCalMm / measured);
          Serial.printf("[C] commanded=%.3f measured=%.3f -> steps/mm %.3f -> %.3f\n",
                        lastCalMm, measured, steps_per_mm, corrected);
          steps_per_mm = corrected;
          applyMotion();
          awaitingCalMeasure = false;
          Serial.println(F("    ^ paste this into 'float steps_per_mm = ...;' so it survives a reset"));
        } else {
          Serial.println(F("[C] send a positive measured number (e.g. 98.7)"));
        }
      } else {
        Serial.printf("[?] unknown: '%s' (send ? for help)\n", s.c_str());
      }
  }
}

// Handle one line from the ESP32-CAM ("QR:<code>" or "NOQR"), §8 protocol.
void handleCamLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.startsWith("QR:")) {
    String code = line.substring(3); code.trim();
    Serial.printf("[CAM] QR:%s\n", code.c_str());
    if (!autosort) return;                       // link test only, no motion
    if (!allHomed()) { Serial.println(F("[CAM] not homed -> QR ignored (HA first)")); return; }
    if (!routeForCode(code.c_str())) { Serial.printf("[CAM] unknown code '%s' -> reject\n", code.c_str()); return; }
    sortCycle(code.c_str());
  } else if (line == "NOQR") {
    Serial.println(F("[CAM] NOQR"));
  } else {
    Serial.printf("[CAM] (raw) %s\n", line.c_str());
  }
}

// ESP-NOW receive callback — keep it SHORT (runs in the WiFi task): just copy the
// message out; the actual work happens in loop() via handleCamLine.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
  if (camMsgReady) return;                       // previous not processed yet -> drop
  int n = (len < 63) ? len : 63;
  memcpy(camMsgBuf, data, n);
  camMsgBuf[n] = '\0';
  camMsgReady = true;
}

void setup() {
  Serial.begin(115200);
  Cam.begin(9600, SERIAL_8N1, CAM_RX_PIN, CAM_TX_PIN);   // legacy UART link (unused; CAM is on ESP-NOW)
  delay(300);
  Serial.println(F("\n=== BRAIN (stage C: motion + homing + gripper + CAM/QR over ESP-NOW) ==="));

  // ESP-NOW: receive QR strings wirelessly from the ESP32-CAM (no wire needed).
  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.printf("ESP-NOW ready (brain). MAC = %s\n", WiFi.macAddress().c_str());
  } else {
    Serial.println(F("ESP-NOW init FAILED"));
  }

  for (int i = 0; i < N_AXES; i++) {
    pinMode(AXES[i].minPin, INPUT_PULLUP);
    pinMode(AXES[i].maxPin, INPUT_PULLUP);
  }

  // SG90 gripper on LEDC (API differs between ESP32 core 2.x and 3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_RES);
#else
  ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
#endif
  servoWrite(grip_open_deg);          // start opened

  engine.init();
  for (int i = 0; i < N_AXES; i++) {
    Axis &a = AXES[i];
    a.s = engine.stepperConnectToPin(a.step);
    if (!a.s) {
      Serial.printf("FATAL: could not init %s stepper\n", a.name);
      while (true) delay(1000);
    }
    a.s->setDirectionPin(a.dir);
    a.s->setEnablePin(ENABLE_PIN);   // shared, active LOW
    a.s->setAutoEnable(true);
    a.s->setCurrentPosition(0);
  }
  applyMotion();

  Serial.println(F("NOT homed yet. Run HA (after confirming Z homes UP/away from the bed)."));
  printHelp();
  printStatus();
}

void loop() {
  // USB console
  static String buf;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length()) { handleLine(buf); buf = ""; }
    } else {
      buf += ch;
    }
  }
  // ESP-NOW: a QR string arrived from the camera (copied in the callback)
  if (camMsgReady) {
    handleCamLine(String(camMsgBuf));
    camMsgReady = false;
  }
}
