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

// Soft limits (usable travel per axis, mm). 0 = unknown/disabled -> only the
// hardware endstops protect that axis. Fill in after measuring real travel.
float soft_max_mm[N_AXES] = { 0, 0, 0 };   // {X, Y, Z}

// Homing parameters
float HOME_FAST_MM_S = 15.0f;      // fast approach
float HOME_SLOW_MM_S = 3.0f;       // slow re-approach (removes bounce error)
float HOME_BACKOFF_MM = 4.0f;      // pull-off between the two approaches
const int8_t HOME_DIR = -1;        // toward MIN = negative (runBackward). Flip DIR
                                   // wiring if an axis runs the wrong way.
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
// Runs an axis continuously toward MIN(dir<0)/MAX(dir>0) until that endstop
// trips (debounced) or timeout. Returns true if the switch was hit.
bool runUntilEndstop(int i, int8_t dir, float speed_mm_per_s, uint32_t timeoutMs) {
  Axis &a = AXES[i];
  uint8_t pin = (dir > 0) ? a.maxPin : a.minPin;
  a.s->setSpeedInHz((uint32_t)lroundf(speed_mm_per_s * steps_per_mm));
  if (dir > 0) a.s->runForward(); else a.s->runBackward();

  uint32_t t0 = millis();
  uint8_t trip = 0;
  while (true) {
    if (endstopTripped(pin)) {
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
  if (soft_max_mm[i] > 0) target_mm = constrain(target_mm, 0.0f, soft_max_mm[i]);
  long target = mmToSteps(target_mm);
  int8_t dir = (target >= a.s->getCurrentPosition()) ? +1 : -1;
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
  Serial.printf("[home] %s ...\n", a.name);

  // if already sitting on the MIN switch, step off it first
  if (endstopTripped(a.minPin)) moveRelBlocking(i, -HOME_DIR * HOME_BACKOFF_MM, HOME_SLOW_MM_S);

  // 1) fast approach toward MIN
  if (!runUntilEndstop(i, HOME_DIR, HOME_FAST_MM_S, HOME_TIMEOUT_MS)) {
    Serial.printf("[home] %s FAILED (no MIN switch within timeout)\n", a.name);
    return false;
  }
  // 2) back off the switch
  moveRelBlocking(i, -HOME_DIR * HOME_BACKOFF_MM, HOME_SLOW_MM_S);
  // 3) slow re-approach
  if (!runUntilEndstop(i, HOME_DIR, HOME_SLOW_MM_S, HOME_TIMEOUT_MS)) {
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
    "BRAIN stage A — motion + homing\n"
    "  H[axis]  home: HA=all(Z->X->Y) / HX / HY / HZ\n"
    "  J<axis><mm>  jog relative  (JX50 / JZ-10)\n"
    "  G<axis><mm>  goto absolute (GX120)\n"
    "  C<axis><mm>  calibrate: jog known dist, then type the MEASURED mm\n"
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
  Serial.println(F("\n=== BRAIN (stage A: motion + homing) ==="));

  for (int i = 0; i < N_AXES; i++) {
    pinMode(AXES[i].minPin, INPUT_PULLUP);
    pinMode(AXES[i].maxPin, INPUT_PULLUP);
  }

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
