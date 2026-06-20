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
float soft_max_mm[N_AXES] = {  194,   165,      0 };   // {X, Y, Z}

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
  // which switch lies in the +position direction depends on where the axis homed:
  // homed at MIN -> +pos goes toward MAX ;  homed at MAX -> +pos goes toward MIN.
  uint8_t posPin = HOME_USES_MAX[i] ? a.minPin : a.maxPin;
  uint8_t negPin = HOME_USES_MAX[i] ? a.maxPin : a.minPin;
  uint8_t ahead  = (dir > 0) ? posPin : negPin;

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
void servoWrite(int deg) { ledcWrite(SERVO_CH, angleToDuty(deg)); }
void gripOpen()  { servoWrite(grip_open_deg);  Serial.println(F("[grip] OPEN")); }
void gripClose() { servoWrite(grip_close_deg); Serial.println(F("[grip] CLOSED")); }

// ── Taught positions (hardcoded; teach with T, dump with D, paste back here) ─
struct Pos { float x, y, z; };          // mm. Z is negative-down (0 = top / safe).
Pos PICK_POS = { 0, 0, 0 };             // TEACH: jog there, TP
Pos SCAN_POS = { 0, 0, 0 };             // TEACH: TS
Pos SHELF[3][3] = {                     // TEACH: T11..T33  (row, col). Col = MS/BV/CJ.
  { {0,0,0}, {0,0,0}, {0,0,0} },
  { {0,0,0}, {0,0,0}, {0,0,0} },
  { {0,0,0}, {0,0,0}, {0,0,0} },
};
const char *COL_PREFIX[3] = { "MS", "BV", "CJ" };   // shelf column = QR prefix

bool allHomed() { return homed[0] && homed[1] && homed[2]; }

Pos curPos() {
  return { stepsToMm(AXES[AX_X].s->getCurrentPosition()),
           stepsToMm(AXES[AX_Y].s->getCurrentPosition()),
           stepsToMm(AXES[AX_Z].s->getCurrentPosition()) };
}

// Resolve a slot token ("P","S","11".."33") to a Pos* and fill its name.
Pos *resolveSlot(const String &a, char *nameOut) {
  if (a.length() == 0) return nullptr;
  char c0 = toupper(a.charAt(0));
  if (c0 == 'P') { strcpy(nameOut, "PICK"); return &PICK_POS; }
  if (c0 == 'S') { strcpy(nameOut, "SCAN"); return &SCAN_POS; }
  if (a.length() >= 2 && isDigit(a.charAt(0)) && isDigit(a.charAt(1))) {
    int r = a.charAt(0) - '1', c = a.charAt(1) - '1';
    if (r >= 0 && r < 3 && c >= 0 && c < 3) { sprintf(nameOut, "SHELF[%d][%d]", r, c); return &SHELF[r][c]; }
  }
  return nullptr;
}

// Safe move to a position: raise Z to top (0), move XY, then lower Z (§7).
void gotoPos(const Pos &p) {
  gotoBlockingMM(AX_Z, 0);        // raise to safe height FIRST (no shelf collisions)
  gotoBlockingMM(AX_X, p.x);
  gotoBlockingMM(AX_Y, p.y);
  gotoBlockingMM(AX_Z, p.z);      // then lower onto the target
}

// Print all taught positions as C code, ready to paste back into this file.
void dumpPositions() {
  Serial.println(F("---- paste into brain.ino ----"));
  Serial.printf("Pos PICK_POS = { %.2f, %.2f, %.2f };\n", PICK_POS.x, PICK_POS.y, PICK_POS.z);
  Serial.printf("Pos SCAN_POS = { %.2f, %.2f, %.2f };\n", SCAN_POS.x, SCAN_POS.y, SCAN_POS.z);
  Serial.println(F("Pos SHELF[3][3] = {"));
  for (int r = 0; r < 3; r++) {
    Serial.print(F("  {"));
    for (int c = 0; c < 3; c++)
      Serial.printf(" {%.2f,%.2f,%.2f}%s", SHELF[r][c].x, SHELF[r][c].y, SHELF[r][c].z, c < 2 ? "," : "");
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
    "  T<slot> teach here   M<slot> move there   D dump positions  (slot: P S 11..33)\n"
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
      gotoBlockingMM(ai, stepsToMm(AXES[ai].s->getCurrentPosition()) + mm);
      Serial.printf("[J] %s now %.2f mm\n", AXES[ai].name, stepsToMm(AXES[ai].s->getCurrentPosition()));
      break;
    }
    case 'G': {
      int ai = axisIndex(arg.charAt(0));
      if (ai < 0) { Serial.println(F("[G] usage: GX120")); break; }
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
      char name[16];
      Pos *slot = resolveSlot(arg, name);
      if (!slot) { Serial.println(F("[T] usage: TP / TS / T11..T33")); break; }
      if (!allHomed()) { Serial.println(F("[T] home first (HA) so positions are referenced")); break; }
      *slot = curPos();
      Serial.printf("[T] %s = {%.2f, %.2f, %.2f}  (use D to dump all for pasting)\n",
                    name, slot->x, slot->y, slot->z);
      break;
    }
    case 'M': {   // move to a taught slot (safe Z motion)
      char name[16];
      Pos *slot = resolveSlot(arg, name);
      if (!slot) { Serial.println(F("[M] usage: MP / MS / M11..M33")); break; }
      if (!allHomed()) { Serial.println(F("[M] home first (HA)")); break; }
      Serial.printf("[M] -> %s {%.2f, %.2f, %.2f}\n", name, slot->x, slot->y, slot->z);
      gotoPos(*slot);
      break;
    }
    case 'D':
      dumpPositions();
      break;
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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== BRAIN (stage B: motion + homing + gripper) ==="));

  for (int i = 0; i < N_AXES; i++) {
    pinMode(AXES[i].minPin, INPUT_PULLUP);
    pinMode(AXES[i].maxPin, INPUT_PULLUP);
  }

  // SG90 gripper on LEDC
  ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
  ledcAttachPin(SERVO_PIN, SERVO_CH);
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
  static String buf;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length()) { handleLine(buf); buf = ""; }
    } else {
      buf += ch;
    }
  }
}
