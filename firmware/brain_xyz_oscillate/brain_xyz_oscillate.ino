/*
 * Brain firmware — X+Y+Z 20mm back-and-forth (oscillation) test
 * -------------------------------------------------------------
 * Board: ESP32-DevKitC WROOM-32D ("the brain"), driving 3 A4988 drivers on a
 * CNC Shield V3. Signals wired underneath to the shield's D2..D8 header pins.
 *
 * On boot all 3 axes set their current position as ZERO. On the S command they
 * oscillate together: 0 -> travel mm -> 0 -> ... at a speed you set live.
 *
 * Wiring (D-pins on the CNC Shield V3 -> ESP32 GPIO):
 *   X STEP D2->25   Y STEP D3->32   Z STEP D4->27
 *   X DIR  D5->26   Y DIR  D6->33   Z DIR  D7->14
 *   EN (all) D8->13 (active LOW)
 *   Shield 5V -> ESP32 5V (driver VDD logic, MUST be 5V), GND common,
 *   12V -> shield power terminal (VMOT, +100uF cap). Vref ~0.8V per driver.
 *   No MS jumpers = FULL STEP -> steps_per_mm = 50 (4mm lead). 1/16 -> 800.
 *
 * ─── Serial commands (115200, newline-terminated), all apply to ALL axes ────
 *   V<mm/s>   set speed         e.g.  V10 / V40
 *   A<mm/s2>  set acceleration  e.g.  A300 / A1500
 *   D<mm>     set distance      e.g.  D20 / D50
 *   S start   X stop   Z zero here   P status   ?  help
 */

#include "FastAccelStepper.h"

// ── Per-axis pins ───────────────────────────────────────────────────────────
struct AxisPins { const char *name; uint8_t step; uint8_t dir; };
const AxisPins AXES[] = {
  { "X", 25, 26 },
  { "Y", 32, 33 },
  { "Z", 27, 14 },
};
const int N_AXES = sizeof(AXES) / sizeof(AXES[0]);
#define ENABLE_PIN 13          // shared A4988 enable, ACTIVE LOW

// ── Calibration & motion (shared by all axes) ───────────────────────────────
float steps_per_mm = 50.0f;    // FULL STEP, 4mm lead. (1/16 microstepping -> 800)
float travel_mm    = 20.0f;    // back-and-forth distance (D command changes it)
float speed_mm_s   = 10.0f;    // default speed; change live with V
float accel_mm_s2  = 600.0f;   // higher default so short 20mm moves reach speed

bool running = false;          // wait for the S command before moving

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stp[N_AXES] = { nullptr };

long mmToSteps(float mm) { return (long)lroundf(mm * steps_per_mm); }

void applyMotion() {
  for (int i = 0; i < N_AXES; i++) {
    if (!stp[i]) continue;
    stp[i]->setSpeedInHz((uint32_t)lroundf(speed_mm_s * steps_per_mm));
    stp[i]->setAcceleration((uint32_t)lroundf(accel_mm_s2 * steps_per_mm));
  }
}

// When an axis is stopped at an endpoint, send it to the other one.
// Position-based & idempotent, so it can't double-trigger a reversal.
void oscillate(FastAccelStepper *s) {
  if (!s || s->isRunning()) return;
  long topSteps = mmToSteps(travel_mm);
  long target = (s->getCurrentPosition() >= topSteps) ? 0 : topSteps;
  s->moveTo(target);
}

void printStatus() {
  Serial.print(F("[P] "));
  for (int i = 0; i < N_AXES; i++)
    Serial.printf("%s=%.2fmm  ", AXES[i].name,
                  (float)stp[i]->getCurrentPosition() / steps_per_mm);
  Serial.printf("| running=%s | dist=%.1f mm | V=%.1f mm/s | A=%.1f mm/s^2 | "
                "steps/mm=%.1f\n", running ? "yes" : "no", travel_mm,
                speed_mm_s, accel_mm_s2, steps_per_mm);
}

void printHelp() {
  Serial.println(F(
    "Commands (apply to ALL axes X/Y/Z):\n"
    "  V<mm/s>  set speed (V10 / V40)\n"
    "  A<mm/s2> set acceleration (A600 / A1500)\n"
    "  D<mm>    set travel distance (D20 / D50)\n"
    "  S  start    X  stop    Z  zero here    P  status    ?  help"));
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
    case 'D':
      travel_mm = max(0.5f, arg.toFloat());
      Serial.printf("[D] travel = %.2f mm\n", travel_mm);
      break;
    case 'S':
      running = true;  Serial.println(F("[S] oscillating X+Y+Z"));
      break;
    case 'X':
      running = false;
      for (int i = 0; i < N_AXES; i++) stp[i]->stopMove();
      Serial.println(F("[X] stopped"));
      break;
    case 'Z':
      for (int i = 0; i < N_AXES; i++) { stp[i]->forceStop(); stp[i]->setCurrentPosition(0); }
      Serial.println(F("[Z] zeroed here (all axes)"));
      break;
    case 'P':
      printStatus();
      break;
    case '?':
      printHelp();
      break;
    default:
      Serial.printf("[?] unknown: '%s' (send ? for help)\n", s.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== X+Y+Z 20mm back-and-forth test ==="));

  engine.init();
  for (int i = 0; i < N_AXES; i++) {
    stp[i] = engine.stepperConnectToPin(AXES[i].step);
    if (!stp[i]) {
      Serial.printf("FATAL: could not init %s stepper (bad STEP pin / engine full)\n",
                    AXES[i].name);
      while (true) delay(1000);
    }
    stp[i]->setDirectionPin(AXES[i].dir);
    stp[i]->setEnablePin(ENABLE_PIN);   // shared, active LOW (A4988)
    stp[i]->setAutoEnable(true);        // enable during moves, disable when idle
    stp[i]->setCurrentPosition(0);      // ZERO is wherever we boot
  }
  applyMotion();

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

  // ── oscillate all axes together ───────────────────────────────────────────
  if (running)
    for (int i = 0; i < N_AXES; i++) oscillate(stp[i]);
}
