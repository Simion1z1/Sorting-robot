/*
 * Brain firmware — X+Y+Z limit-to-limit bounce test (endstops)
 * -----------------------------------------------------------
 * Board: ESP32-DevKitC WROOM-32D ("the brain"), 3 A4988 drivers on a CNC
 * Shield V3. Each axis runs continuously toward one limit switch; when it
 * trips the switch it stops, backs off a few mm, REVERSES, and runs to the
 * other limit. So every axis ping-pongs between its MIN and MAX endstops.
 *
 * Unlike brain_xyz_oscillate (fixed 20mm travel), here the travel is decided
 * by the hardware limit switches — the move ends only when a switch trips.
 *
 * ─── Wiring — STEP/DIR/EN identical to brain_xyz_oscillate ──────────────────
 *   X STEP D2->25   Y STEP D3->32   Z STEP D4->27
 *   X DIR  D5->26   Y DIR  D6->33   Z DIR  D7->14
 *   EN (all) D8->13 (active LOW)
 *   Shield 5V -> ESP32 5V (driver VDD logic), GND common, 12V -> VMOT.
 *   No MS jumpers = FULL STEP -> steps_per_mm = 50 (4mm lead). 1/16 -> 800.
 *
 * ─── Endstops (per CLAUDE.md §4.2) — NC contact COM->GND, signal->GPIO ──────
 *   X MIN 21   Y MIN 22   Z MIN 23      (home end)
 *   X MAX  4   Y MAX 18   Z MAX 17      (far end)
 *   All read with INPUT_PULLUP. NC wiring => closed/at-rest = LOW,
 *   tripped OR broken wire = HIGH (fail-safe). See ENDSTOP_TRIPPED_LEVEL.
 *
 *   !!! If a switch is NOT wired, INPUT_PULLUP makes that pin read HIGH = it
 *       looks permanently tripped and the axis will jitter in place. Either
 *       wire all 6 switches, or comment out the axes you aren't testing.
 *
 * ─── Serial commands (115200, newline-terminated), apply to ALL axes ────────
 *   V<mm/s>   set speed         e.g.  V10 / V40
 *   A<mm/s2>  set acceleration  e.g.  A300 / A1500
 *   B<mm>     set back-off dist e.g.  B3  / B5
 *   S start   X stop   Z zero here   P status   ?  help
 */

#include "FastAccelStepper.h"

// ── Endstop electrical convention ───────────────────────────────────────────
// Wiring used here: NO (Normally Open) terminal + INPUT_PULLUP, COM->GND.
//   at rest  = contact open  -> pin pulled HIGH -> NOT tripped
//   pressed  = contact closed-> pin to GND LOW  -> TRIPPED
// So with NO wiring, TRIPPED = LOW.
// (If you move the wires to the NC terminal instead -> set this to HIGH; NC is
//  fail-safe: a broken wire then reads as tripped. Recommended for the final robot.)
#define ENDSTOP_TRIPPED_LEVEL LOW
#define DEBOUNCE_SAMPLES 4     // consecutive tripped reads before we believe it

// ── Per-axis pins ───────────────────────────────────────────────────────────
struct Axis {
  const char *name;
  uint8_t step, dir;
  uint8_t minPin, maxPin;          // MIN = home end, MAX = far end
  FastAccelStepper *s;
  int8_t  travelDir;               // +1 = toward MAX, -1 = toward MIN
  uint8_t state;                   // ST_RUN / ST_BACKOFF
  uint8_t trip;                    // debounce counter
};

enum { ST_RUN = 0, ST_BACKOFF = 1 };

Axis AXES[] = {
  //  name  step dir  min max
  { "X", 25, 26,  21,  4,  nullptr, +1, ST_RUN, 0 },
  { "Y", 32, 33,  22, 18,  nullptr, +1, ST_RUN, 0 },
  { "Z", 27, 14,  23, 17,  nullptr, +1, ST_RUN, 0 },
};
const int N_AXES = sizeof(AXES) / sizeof(AXES[0]);
#define ENABLE_PIN 13              // shared A4988 enable, ACTIVE LOW

// ── Calibration & motion (shared by all axes) ───────────────────────────────
float steps_per_mm = 50.0f;        // FULL STEP, 4mm lead. (1/16 microstepping -> 800)
float speed_mm_s   = 10.0f;        // change live with V
float accel_mm_s2  = 600.0f;
float backoff_mm   = 3.0f;         // how far to pull off the switch before reversing

bool running = false;              // wait for the S command before moving

// ── calibration / single-axis jog state (only used when running == false) ───
int    jogAxis = -1;               // axis currently jogging (-1 = none)
int8_t jogDir  = 0;                // +1 toward MAX, -1 toward MIN (limit safety)
float  lastCalMm = 0.0f;           // commanded distance of the last C move
bool   awaitingCalMeasure = false; // true after a C move, until you send the measured mm

FastAccelStepperEngine engine = FastAccelStepperEngine();

long mmToSteps(float mm) { return (long)lroundf(mm * steps_per_mm); }

// map an axis letter ('X'/'Y'/'Z') to its index, or -1 if unknown
int axisIndex(char c) {
  c = toupper(c);
  for (int i = 0; i < N_AXES; i++) if (AXES[i].name[0] == c) return i;
  return -1;
}

// Debounced endstop read: returns true only after DEBOUNCE_SAMPLES tripped reads.
bool endstopTripped(uint8_t pin) {
  return digitalRead(pin) == ENDSTOP_TRIPPED_LEVEL;
}

void applyMotion() {
  for (int i = 0; i < N_AXES; i++) {
    if (!AXES[i].s) continue;
    AXES[i].s->setSpeedInHz((uint32_t)lroundf(speed_mm_s * steps_per_mm));
    AXES[i].s->setAcceleration((uint32_t)lroundf(accel_mm_s2 * steps_per_mm));
  }
}

// Drive one axis continuously in its current travel direction.
void runAxis(Axis &a) {
  if (a.travelDir > 0) a.s->runForward();
  else                 a.s->runBackward();
}

// Per-axis bounce logic: run toward the limit in travelDir; on trip back off
// and reverse, so it ping-pongs between MIN and MAX forever.
void serviceAxis(Axis &a) {
  if (!a.s) return;

  switch (a.state) {
    case ST_RUN: {
      uint8_t ahead = (a.travelDir > 0) ? a.maxPin : a.minPin;

      // debounce: count consecutive tripped reads
      if (endstopTripped(ahead)) {
        if (a.trip < 255) a.trip++;
      } else {
        a.trip = 0;
      }

      if (a.trip >= DEBOUNCE_SAMPLES) {
        a.s->forceStop();                       // hit the limit -> stop now
        a.travelDir = -a.travelDir;             // reverse
        a.s->move((long)a.travelDir * mmToSteps(backoff_mm));  // pull off the switch
        a.trip = 0;
        a.state = ST_BACKOFF;
        Serial.printf("[%s] limit %s -> reverse, now heading %s\n", a.name,
                      (ahead == a.maxPin) ? "MAX" : "MIN",
                      (a.travelDir > 0) ? "MAX" : "MIN");
      } else if (!a.s->isRunning()) {
        runAxis(a);                             // keep it moving toward the limit
      }
      break;
    }

    case ST_BACKOFF:
      // wait for the back-off move to finish, then resume continuous run.
      if (!a.s->isRunning()) {
        a.state = ST_RUN;
        runAxis(a);
      }
      break;
  }
}

void stopAll() {
  running = false;
  jogAxis = -1;
  awaitingCalMeasure = false;
  for (int i = 0; i < N_AXES; i++) {
    if (!AXES[i].s) continue;
    AXES[i].s->forceStop();
    AXES[i].state = ST_RUN;
    AXES[i].trip = 0;
  }
}

// Start a single-axis relative move (jog). Used for jogging & calibration.
// Refuses while the bounce is running. Stops automatically if the limit in
// the direction of travel trips (see loop()).
void startJog(int ai, float mm) {
  if (running) { Serial.println(F("[!] stop the bounce first (send X)")); return; }
  if (ai < 0)  { Serial.println(F("[!] usage: pick axis X/Y/Z, e.g. JX50 / CZ100")); return; }
  jogAxis = ai;
  jogDir  = (mm >= 0) ? +1 : -1;
  AXES[ai].s->move(mmToSteps(mm));
  Serial.printf("[jog] %s %.3f mm (%ld steps)\n", AXES[ai].name, mm, mmToSteps(mm));
}

void printStatus() {
  Serial.print(F("[P] "));
  for (int i = 0; i < N_AXES; i++) {
    Axis &a = AXES[i];
    Serial.printf("%s=%.1fmm(%s%c%c) ", a.name,
                  (float)a.s->getCurrentPosition() / steps_per_mm,
                  a.travelDir > 0 ? "->MAX" : "->MIN",
                  endstopTripped(a.minPin) ? 'm' : '-',   // m = MIN tripped
                  endstopTripped(a.maxPin) ? 'M' : '-');   // M = MAX tripped
  }
  Serial.printf("| running=%s | V=%.1f mm/s | A=%.1f mm/s^2 | backoff=%.1f mm | steps/mm=%.3f\n",
                running ? "yes" : "no", speed_mm_s, accel_mm_s2, backoff_mm, steps_per_mm);
}

void printHelp() {
  Serial.println(F(
    "Limit-to-limit bounce test (apply to ALL axes X/Y/Z):\n"
    "  V<mm/s>  set speed (V10 / V40)\n"
    "  A<mm/s2> set acceleration (A600 / A1500)\n"
    "  B<mm>    set back-off distance (B3 / B5)\n"
    "  J<axis><mm> jog ONE axis (JX50 / JY-20 / JZ100) -- bounce must be stopped\n"
    "  C<axis><mm> CALIBRATE: jog a known dist, then send the MEASURED mm\n"
    "              e.g.  CX100  -> measure with calipers -> type 98.7\n"
    "  S  start    X  stop    Z  zero here    P  status    ?  help\n"
    "  Legend in P: m=MIN tripped, M=MAX tripped"));
}

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = toupper(s.charAt(0));
  String arg = s.substring(1); arg.trim();

  switch (c) {
    case 'V':
      speed_mm_s = max(0.1f, arg.toFloat()); applyMotion();
      Serial.printf("[V] speed = %.2f mm/s\n", speed_mm_s);
      break;
    case 'A':
      accel_mm_s2 = max(1.0f, arg.toFloat()); applyMotion();
      Serial.printf("[A] accel = %.2f mm/s^2\n", accel_mm_s2);
      break;
    case 'B':
      backoff_mm = max(0.5f, arg.toFloat());
      Serial.printf("[B] back-off = %.2f mm\n", backoff_mm);
      break;
    case 'J': {   // jog one axis: J<axis><mm>, e.g. JX50 / JZ-10
      int ai = axisIndex(arg.charAt(0));
      startJog(ai, arg.substring(1).toFloat());
      break;
    }
    case 'C': {   // calibrate one axis: C<axis><mm>, then send measured mm
      int ai = axisIndex(arg.charAt(0));
      float mm = arg.substring(1).toFloat();
      if (ai < 0 || mm == 0) { Serial.println(F("[C] usage: CX100  (then measure travel, then type the measured mm)")); break; }
      lastCalMm = fabs(mm);
      awaitingCalMeasure = true;
      startJog(ai, mm);
      Serial.println(F("[C] when it stops, MEASURE the travel with calipers, then type just the number (e.g. 98.7)"));
      break;
    }
    case 'S':
      running = true;
      jogAxis = -1; awaitingCalMeasure = false;
      for (int i = 0; i < N_AXES; i++) { AXES[i].state = ST_RUN; AXES[i].trip = 0; }
      Serial.println(F("[S] bouncing X+Y+Z between their limits"));
      break;
    case 'X':
      stopAll();
      Serial.println(F("[X] stopped"));
      break;
    case 'Z':
      for (int i = 0; i < N_AXES; i++) { AXES[i].s->forceStop(); AXES[i].s->setCurrentPosition(0); }
      Serial.println(F("[Z] zeroed here (all axes)"));
      break;
    case 'P':
      printStatus();
      break;
    case '?':
      printHelp();
      break;
    default:
      // While awaiting a calibration measurement, a bare number = the measured mm.
      if (awaitingCalMeasure) {
        float measured = s.toFloat();
        if (measured > 0) {
          float corrected = steps_per_mm * (lastCalMm / measured);
          Serial.printf("[C] commanded=%.3f measured=%.3f -> steps/mm %.3f -> %.3f\n",
                        lastCalMm, measured, steps_per_mm, corrected);
          steps_per_mm = corrected;
          applyMotion();
          awaitingCalMeasure = false;
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
  Serial.println(F("\n=== X+Y+Z limit-to-limit bounce test ==="));

  // endstop inputs (NC + internal pull-up, no external resistors)
  for (int i = 0; i < N_AXES; i++) {
    pinMode(AXES[i].minPin, INPUT_PULLUP);
    pinMode(AXES[i].maxPin, INPUT_PULLUP);
  }

  engine.init();
  for (int i = 0; i < N_AXES; i++) {
    Axis &a = AXES[i];
    a.s = engine.stepperConnectToPin(a.step);
    if (!a.s) {
      Serial.printf("FATAL: could not init %s stepper (bad STEP pin / engine full)\n", a.name);
      while (true) delay(1000);
    }
    a.s->setDirectionPin(a.dir);
    a.s->setEnablePin(ENABLE_PIN);     // shared, active LOW (A4988)
    a.s->setAutoEnable(true);          // enable during moves, disable when idle
    a.s->setCurrentPosition(0);
  }
  applyMotion();

  // warn about any switch that is already tripped (likely not wired)
  for (int i = 0; i < N_AXES; i++) {
    if (endstopTripped(AXES[i].minPin))
      Serial.printf("WARN: %s MIN reads tripped at boot (unwired? check NC->GND)\n", AXES[i].name);
    if (endstopTripped(AXES[i].maxPin))
      Serial.printf("WARN: %s MAX reads tripped at boot (unwired? check NC->GND)\n", AXES[i].name);
  }

  printHelp();
  printStatus();
}

void loop() {
  // ── serial command parsing ────────────────────────────────────────────────
  static String buf;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buf.length()) { handleLine(buf); buf = ""; }
    } else {
      buf += ch;
    }
  }

  // ── bounce all axes between their limits ──────────────────────────────────
  if (running) {
    for (int i = 0; i < N_AXES; i++) serviceAxis(AXES[i]);
  }
  // ── single-axis jog / calibration (only when not bouncing) ────────────────
  else if (jogAxis >= 0) {
    Axis &a = AXES[jogAxis];
    uint8_t ahead = (jogDir > 0) ? a.maxPin : a.minPin;
    if (endstopTripped(ahead)) {            // safety: never crash into the limit
      a.s->forceStop();
      Serial.printf("[jog] %s hit %s limit -> stopped\n", a.name, jogDir > 0 ? "MAX" : "MIN");
      jogAxis = -1;
      awaitingCalMeasure = false;           // travel was cut short -> measurement invalid
    } else if (!a.s->isRunning()) {
      jogAxis = -1;                         // jog finished cleanly
    }
  }
}
