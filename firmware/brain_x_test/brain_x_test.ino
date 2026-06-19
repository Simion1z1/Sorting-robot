/*
 * Brain firmware — STEP 2 (bench): X axis only
 * ----------------------------------------------
 * Target board: ESP32-DevKitC WROOM-32D ("the brain", §1).
 * Scope of THIS sketch: drive ONLY the X stepper through its A4988, using
 * FastAccelStepper. No homing, no endstops, no Y/Z — those pins are left alone.
 * (Maps to Plan §9 Stage 5 / step 28: "each axis does a clean move".)
 *
 * Pin map per IMPLEMENTATION_PLAN §4.2:
 *   X STEP = GPIO25,  X DIR = GPIO26,  ENABLE (shared A4988) = GPIO13 (active LOW)
 *
 * A4988 wiring reminder (§4.4): VMOT+100µF→12V, VDD→3.3V, RESET tied to SLEEP,
 * MS1/MS2/MS3 by jumpers (e.g. all HIGH = 1/16). Set Vref before powering motor.
 *
 * Library: "FastAccelStepper" by gin66 — install via Library Manager.
 *
 * ─── Serial command interface (115200, newline-terminated) ──────────────────
 *   M<mm>   relative move,   e.g.  M50   /  M-20.5
 *   G<mm>   absolute move,   e.g.  G0    /  G100
 *   V<mm/s> set max speed,   e.g.  V15
 *   A<mm/s2>set acceleration,e.g.  A300
 *   Z       set current position as zero (origin)
 *   P       print position (steps + mm) and settings
 *   X       STOP now (decelerate)
 *   N1 / N0 eNable / disable the driver (N0 lets you turn the screw by hand)
 *   C<mm>   calibration helper: command a known move, then measure with calipers
 *   ?       help
 */

#include "FastAccelStepper.h"

// ── Pins (brain WROOM-32D, §4.2) ────────────────────────────────────────────
#define X_STEP_PIN   25
#define X_DIR_PIN    26
#define ENABLE_PIN   13      // shared A4988 enable, ACTIVE LOW

// ── Calibration ─────────────────────────────────────────────────────────────
// steps_per_mm = (motor_steps × microstepping) / screw_lead_mm   (§5.2)
// FULL STEP (no MS jumpers on the CNC Shield): 200 × 1 / 4mm lead = 50.
// (If you later add all 3 MS jumpers = 1/16, change this to 800.)
// ADJUST after the C-command measurement — the real screw lead may differ!
float steps_per_mm = 50.0f;

// ── Conservative motion defaults (mm units, converted to steps below) ───────
float max_speed_mm_s = 10.0f;    // start slow; raise later (§10)
float accel_mm_s2    = 200.0f;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *X = nullptr;

// ── helpers ─────────────────────────────────────────────────────────────────
long mmToSteps(float mm) { return (long)lroundf(mm * steps_per_mm); }
float stepsToMm(long s)  { return (float)s / steps_per_mm; }

void applyMotionSettings() {
  if (!X) return;
  X->setSpeedInHz((uint32_t)lroundf(max_speed_mm_s * steps_per_mm));
  X->setAcceleration((uint32_t)lroundf(accel_mm_s2 * steps_per_mm));
}

void printStatus() {
  Serial.printf("[X] pos=%ld steps (%.3f mm) | running=%s | "
                "steps/mm=%.3f | Vmax=%.1f mm/s | A=%.1f mm/s^2\n",
                X->getCurrentPosition(), stepsToMm(X->getCurrentPosition()),
                X->isRunning() ? "yes" : "no",
                steps_per_mm, max_speed_mm_s, accel_mm_s2);
}

void printHelp() {
  Serial.println(F(
    "Commands:\n"
    "  M<mm>  relative move (M50 / M-20.5)\n"
    "  G<mm>  absolute move (G0 / G100)\n"
    "  V<mm/s> set max speed   A<mm/s2> set accel\n"
    "  Z  zero here    P  status    X  stop\n"
    "  N1/N0  enable/disable driver\n"
    "  C<mm>  calibrate: moves <mm>, then you measure & we fix steps/mm\n"
    "  ?  this help"));
}

// remember the last commanded distance for the C calibration flow
float lastCalMm = 0.0f;
bool  awaitingCalMeasure = false;

void handleLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = toupper(s.charAt(0));
  String arg = s.substring(1);
  arg.trim();

  switch (c) {
    case 'M': {
      float mm = arg.toFloat();
      X->move(mmToSteps(mm));
      Serial.printf("[M] relative %.3f mm (%ld steps)\n", mm, mmToSteps(mm));
      break;
    }
    case 'G': {
      float mm = arg.toFloat();
      X->moveTo(mmToSteps(mm));
      Serial.printf("[G] absolute -> %.3f mm (%ld steps)\n", mm, mmToSteps(mm));
      break;
    }
    case 'V': {
      max_speed_mm_s = max(0.1f, arg.toFloat());
      applyMotionSettings();
      Serial.printf("[V] max speed = %.2f mm/s\n", max_speed_mm_s);
      break;
    }
    case 'A': {
      accel_mm_s2 = max(1.0f, arg.toFloat());
      applyMotionSettings();
      Serial.printf("[A] accel = %.2f mm/s^2\n", accel_mm_s2);
      break;
    }
    case 'Z':
      X->setCurrentPosition(0);
      Serial.println(F("[Z] position zeroed"));
      break;
    case 'P':
      printStatus();
      break;
    case 'X':
      X->stopMove();
      Serial.println(F("[X] stopping"));
      break;
    case 'N':
      if (arg.toInt() == 1) { X->enableOutputs(); Serial.println(F("[N] driver ENABLED")); }
      else                  { X->disableOutputs(); Serial.println(F("[N] driver DISABLED (free to turn by hand)")); }
      break;
    case 'C': {
      // Calibration: command a known move; afterwards user measures real travel.
      float mm = arg.toFloat();
      if (mm == 0) { Serial.println(F("[C] usage: C100  (then measure travel, then send the measured mm)")); break; }
      lastCalMm = mm;
      awaitingCalMeasure = true;
      X->move(mmToSteps(mm));
      Serial.printf("[C] commanded %.3f mm. When it stops, MEASURE travel with calipers,\n"
                    "    then send just the measured number (e.g. 98.7) to correct steps/mm.\n", mm);
      break;
    }
    case '?':
      printHelp();
      break;
    default:
      // If we're waiting for a calibration measurement, treat a bare number as mm.
      if (awaitingCalMeasure) {
        float measured = s.toFloat();
        if (measured > 0) {
          float corrected = steps_per_mm * (lastCalMm / measured);
          Serial.printf("[C] commanded=%.3f measured=%.3f  ->  steps/mm %.3f -> %.3f\n",
                        lastCalMm, measured, steps_per_mm, corrected);
          steps_per_mm = corrected;
          applyMotionSettings();
          awaitingCalMeasure = false;
        }
      } else {
        Serial.printf("[?] unknown: '%s'  (send ? for help)\n", s.c_str());
      }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== Brain X-axis bench test ==="));

  engine.init();
  X = engine.stepperConnectToPin(X_STEP_PIN);
  if (!X) {
    Serial.println(F("FATAL: could not init stepper (bad STEP pin / engine full)"));
    while (true) delay(1000);
  }
  X->setDirectionPin(X_DIR_PIN);
  X->setEnablePin(ENABLE_PIN);   // FastAccelStepper default = active LOW (matches A4988)
  X->setAutoEnable(true);        // enable on move, disable when idle
  applyMotionSettings();

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
