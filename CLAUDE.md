# CLAUDE.md — XYZ Gantry Pick & Place Robot with QR Sorting

> Cartesian 3-axis robot (linear screw actuators + NEMA 17) that picks small boxes from a feed zone, scans their QR code with an ESP32-CAM mounted on the gripper, and sorts them onto shelves based on the destination encoded in the QR (e.g. `MS`, `BV`, `CJ`).

This file is the project's working spec / context document for the build. Level: advanced. It assumes you can solder, flash an ESP32, and know Arduino basics. It focuses on architecture, design decisions, and implementation steps — not introductory concepts.

---

## 1. System architecture

Two microcontrollers, clear separation of responsibilities:

- **ESP32-CAM (AI-Thinker) = vision node.** Its only job: capture the image, decode the QR (quirc), send the decoded string over UART to the brain. It controls nothing mechanical. A "smart sensor" on the gripper.
- **ESP32-DevKitC (WROOM-32D) = the brain.** Receives the QR string over serial, runs the state machine, generates step pulses for the 3 steppers through the A4988 drivers, homes on the endstops, drives the gripper's SG90 servo, and decides the destination shelf.

```mermaid
flowchart LR
    subgraph Z_Carriage["Z carriage (moving)"]
        CAM["ESP32-CAM + OV2640<br/>QR decode"]
        SERVO["SG90 — parallel gripper"]
    end
    CAM -- "UART 3.3V<br/>QR:CJ0012\\n" --> BRAIN
    BRAIN["ESP32 WROOM-32D<br/>(brain / state machine)"]
    BRAIN -- "PWM (LEDC)" --> SERVO
    BRAIN -- "STEP/DIR" --> DRVX["A4988 X"] --> MX["NEMA17 X"]
    BRAIN -- "STEP/DIR" --> DRVY["A4988 Y"] --> MY["NEMA17 Y"]
    BRAIN -- "STEP/DIR" --> DRVZ["A4988 Z"] --> MZ["NEMA17 Z"]
    ESX["Endstop X SS-5GL"] --> BRAIN
    ESY["Endstop Y SS-5GL"] --> BRAIN
    ESZ["Endstop Z SS-5GL"] --> BRAIN
    PSU["12V PSU"] --> DRVX & DRVY & DRVZ
    PSU --> BUCK["Buck 12V→3.3V/5V"] --> CAM & BRAIN & SERVO
```

**Why two boards and not one:** the ESP32-CAM has nearly all pins taken by the OV2640 camera + PSRAM, so it can't also drive 3 steppers + endstops. QR decoding with quirc is also CPU/RAM heavy — you want the motion loop (timing-critical) to never be blocked by image processing. The split is architecturally correct.

---

## 2. Key design decisions (read before ordering parts)

### 2.1 A4988 vs DRV8825 for a 1.7 A NEMA 17 — caution
The motor label says **1.7 A / phase**. The A4988 realistically delivers ~1 A without a heatsink and ~1.4–1.5 A with a heatsink + airflow; 2 A is the absolute theoretical limit. You will therefore run the motors **under-current** (~1.0–1.2 A), which reduces torque. For a light pick & place (small boxes, moderate speeds) that's acceptable, but:

- **Recommendation:** if you want torque/speed margin, swap the A4988 for a **DRV8825** (up to 2.2 A, 1/32 microstepping, pin-compatible on most StepStick modules). Same cost. If you stick with the A4988 (as requested), put a heatsink on each driver and accept reduced current.
- Current is set via **Vref** (see §6.2 / Phase 2). Do not skip this step — an untuned A4988 means lost steps or burned drivers.

### 2.2 Powering the ESP32-CAM — failure source #1 (brownout)
The ESP32-CAM draws current peaks when the camera/Wi-Fi start (300–500 mA, higher spikes). A weak buck → brownout → boot loop.

- **For the brain (WROOM-32D):** a 12V→3.3V buck into the `3V3` pin is fine (bypasses the onboard LDO). Alternatively 12V→5V into `5V/VIN`.
- **For the ESP32-CAM:** **prefer 12V→5V** in a dedicated buck, connected to the module's `5V` pin (uses the onboard AMS1117 LDO, which handles inrush better). If you insist on 3.3V directly (as requested), use a **≥1 A** buck and add a **470–1000 µF electrolytic + 100 nF ceramic** capacitor right next to the CAM's power pins. Do not power the CAM on both 5V and 3V3 at the same time.
- Common ground (GND) is mandatory between the two bucks, the two ESP32s, and the 12V PSU. Otherwise UART won't work.

### 2.3 QR decoding on the ESP32-CAM — realistic, but with limits
The ESP32-CAM with the `ESP32QRCodeReader` library (quirc-based) works, but the OV2640 has **fixed focus** (~10–25 cm usable range) and is light-sensitive. For reliability:

- QR codes **printed clearly, high contrast**, at least ~25×25 mm at the chosen scan distance.
- Add **illumination** (the GPIO4 LED is too strong/close; a small diffuse side LED is better).
- Fixed camera-to-box distance (the gripper always brings the camera to the same scan position → constant focus).
- Working resolution: `FRAMESIZE_QVGA`/`VGA` is enough for QR; larger = slower.

**More robust alternatives (optional, if QR-on-CAM is troublesome):**
- **Useful Sensors Tiny Code Reader** — I2C module, decodes QR onboard, sends the string directly. Zero processing on the ESP32. Simplest.
- **GM65 / GM861 barcode scanner** — UART module, very reliable, also reads 1D codes. Use it in place of the ESP32-CAM and wire UART straight to the brain.

Recommendation: start with the ESP32-CAM (it's what you have), but if the read rate is below ~90%, switch to the Tiny Code Reader. The serial protocol in §8 stays identical, only the source of the string changes.

### 2.4 Who controls the SG90 gripper
Recommendation: the **brain** drives the SG90 (PWM via LEDC), not the ESP32-CAM. All motion logic (open/close synchronized with the Z descent) stays in one place → clean state machine. Cost: one extra signal wire through the drag chain to the carriage. Worth it.

### 2.5 Stepper voltage 12V
At 12V, the NEMA 17 gives good torque at low–medium speeds; at high speeds torque drops (coil inductance). Fine for pick & place. If you later want speed, the A4988/DRV8825 support up to 35V/45V on the motor — you could move the motors to 24V while keeping the logic on 12→3.3V. For now we stay at 12V per your parts list.

---

## 3. Bill of materials

**Already have (from the brief):**

| Component | Role | Qty |
|---|---|---|
| Linear screw actuator + NEMA17 | X, Y, Z axes | 3 |
| NEMA 17 (1.7 A, 1.8°, 42×42×40) | Stepper motors | 3 (in actuators) |
| A4988 driver | Stepper control | 3 |
| SS-5GL mechanical endstop (5A 125VAC) | Homing / limit | 3 (min.) |
| 12V PSU | Power | 1 |
| ESP32-CAM + ESP32-CAM-MB (HW-381) | Vision / QR | 1 |
| SG90 servo | Gripper actuation | 1 |
| Parallel gripper (GrabCAD) | Box grasping | 1 |
| ESP32-DevKitC WROOM-32D | Brain | 1 |
| Buck 12V→3.3V | Logic power | 1–2 |

**To add (required, not in the list):**

- **Heatsinks** for the 3 A4988s (mandatory at this current).
- **100 µF / 35V electrolytic capacitors** on each A4988's VMOT input (protects the driver from spikes — the driver can burn out without them). Plus 470–1000 µF on the ESP32-CAM power.
- **Dedicated 12V→5V buck** for the ESP32-CAM (see §2.2). Recommended separate from the brain's.
- **6 endstops total**: 3 for homing (MIN, one per axis on GPIO 21/22/23) + 3 end-of-travel (MAX) wired **in series on a single pin GPIO4** with the internal pull-up — no extra resistors (see §4.2).
- **Drag chain (cable carrier)** + flexible wire for the cabling to the moving Z carriage.
- **Connectors** (Dupont/JST), **terminal block**, **fuse** on the 12V (e.g. 5A).
- **Frame / extrusion** for the orthogonal mounting of the 3 actuators (see §5.1) if the actuators don't come with a mounting kit.
- **Correctly sized 12V PSU:** 3× ~1.2A (steppers) + camera/logic ≈ **minimum 5A, recommended 6–8A** with margin.

---

## 4. Electrical architecture & pin map

### 4.1 Power distribution
```
12V PSU ────┬── VMOT A4988 x3 (+ 100µF cap each)
            ├── Buck 12V→3.3V ── 3V3 ESP32 WROOM (brain)
            └── Buck 12V→5V   ── 5V ESP32-CAM (+ 470µF cap)
Common GND for EVERYTHING.
```

### 4.2 Pin map — ESP32 WROOM-32D (brain)
Intentionally avoided: GPIO 0/2/12/15 (strapping), 6–11 (flash SPI), 34–39 (input-only).

| Function | GPIO | Notes |
|---|---|---|
| X STEP | 25 | |
| X DIR | 26 | |
| Y STEP | 32 | |
| Y DIR | 33 | |
| Z STEP | 27 | |
| Z DIR | 14 | |
| ENABLE (shared across all 3 A4988) | 13 | active LOW |
| Endstop X — home/MIN | 21 | `INPUT_PULLUP`, contact to GND |
| Endstop Y — home/MIN | 22 | `INPUT_PULLUP` |
| Endstop Z — home/MIN | 23 | `INPUT_PULLUP` |
| Endstops MAX (X+Y+Z, 3× in series) | 4 | `INPUT_PULLUP`; the 3 NC switches daisy-chained between GPIO4 and GND, no resistors |
| Servo SG90 (PWM) | 19 | LEDC channel, 50 Hz |
| UART2 RX (from CAM) | 16 | `Serial2` |
| UART2 TX (trigger to CAM, optional) | 17 | |
| Status LED (optional) | 2 | strapping — LED output only |

**MS1/MS2/MS3** (microstepping) — **NOT** on GPIO; set them with jumpers on the A4988 board (e.g. all HIGH = 1/16). Saves 9 pins.

> **MAX endstops — 3 in series on GPIO4 (no resistors):** the 3 NC end-of-travel switches are daisy-chained in series between GPIO4 and GND, read with the internal `INPUT_PULLUP`. All closed = LOW; any switch opening (limit hit) or a broken wire = HIGH → fault/stop (fail-safe). You learn *that* a MAX limit tripped, not *which* axis — fine for a safety stop backed by the soft limits. The MAX chain is optional. (Per-axis MAX would instead need the input-only pins 34/35/36 + an external 10k pull-up each.)

### 4.3 Pin map — ESP32-CAM (AI-Thinker)
Camera + PSRAM take most pins. Usable free: GPIO 12, 13, 14, 15, 2, 4, 16. **Strapping caution:** GPIO12 must be LOW at boot, 15 and 2 have constraints.

| Function | GPIO | Notes |
|---|---|---|
| UART TX → brain (`QR:...`) | 13 | the important direction (CAM→brain) |
| "Scan now" trigger ← brain | 14 | input; or continuous scan (see §8) |
| (Optional illumination LED) | 4 | onboard white LED — bright, use briefly |

> Do not use GPIO12 as TX (idle = HIGH would block boot). That's why the CAM's TX is on GPIO13. Connect **common GND** between CAM and brain — both are 3.3V logic, so UART direct, no level shifter.

### 4.4 A4988 wiring (per driver)
```
VMOT, motor-GND ── 12V (+ 100µF cap)
VDD, logic-GND  ── 3.3V from brain
STEP, DIR       ── brain GPIO (table 4.2)
ENABLE          ── shared GPIO13
RESET ── SLEEP   (tied together, to VDD)
1A,1B,2A,2B     ── motor coils (verify pairs with an ohmmeter!)
MS1/MS2/MS3     ── microstepping jumpers
```

---

## 5. Mechanics, coordinate system and kinematics

### 5.1 Orthogonal mounting
Typical cartesian gantry layout:
- **Y axis** (or X) — the base actuator, long, defines the table length.
- **X axis** — mounted perpendicular on the Y carriage (the gantry bridge).
- **Z axis** — mounted vertically on the X carriage; the gripper + ESP32-CAM sit on the Z carriage.

The load on Z (gripper + SG90 + ESP32-CAM + cabling) must stay below the Z motor's torque. Balance the cabling, use the drag chain so it doesn't pull on the carriage.

### 5.2 steps/mm — the fundamental calibration
```
steps_per_mm = (motor_steps × microstepping) / screw_lead_mm
```
Example: 1.8° motor = 200 steps/rev, 1/16 microstepping, 4 mm lead screw:
```
steps_per_mm = (200 × 16) / 4 = 800 steps/mm
```
- **Lead** depends on your actuator (ballscrew 1204 = 4 mm; 1605 = 5 mm; T8 leadscrew = 2/4/8 mm). **Check physically** on the real actuator.
- Empirical calibration: command a 100 mm move, measure with calipers, adjust `steps_per_mm = steps_per_mm × (commanded / measured)`. Repeat per axis.

### 5.3 Homing (endstop referencing)
- Recommended endstop wiring: **Normally Closed (NC)** to GND, read with `INPUT_PULLUP`. NC is safer — a broken wire reads as "triggered" (fail-safe) rather than being missed.
- **Homing sequence (anti-collision order):**
  1. **Z first**, upward, until the endstop → Z = 0 (safe, raised position).
  2. **X** to its endstop → X = 0.
  3. **Y** to its endstop → Y = 0.
- For each axis: fast approach → on contact stop → back off ~2–5 mm → slow re-approach → set zero. This removes mechanical debounce error.
- Add software **debounce** (5–10 ms) on the endstop reads.
- **Homing references the MIN switches (GPIO 21/22/23).** The MAX switches are a hard end-of-travel limit on the GPIO4 series chain: read that one pin in the motion loop and stop/fault on trip (it does not tell you which axis).
- Define **soft limits** (software limits in mm) per axis, so you don't force the end of travel even before the MAX chain trips.

### 5.4 Step-generation library
Use **FastAccelStepper** (optimized for the ESP32: timing on hardware RMT/MCPWM, up to ~200 kHz, smooth motion for all 3 axes simultaneously). Avoid pure-software AccelStepper on the ESP32 for 3 motors at high microstepping — it saturates the CPU and loses steps above ~4 kHz/motor. Set acceleration/max-speed profiles per axis.

---

## 6. Sorting logic (QR → shelf mapping)

### 6.1 Shelf model
From the sketch, the shelves are a 3×3 grid of destinations, encoded by county:

| | Col. 1 | Col. 2 | Col. 3 |
|---|---|---|---|
| Row 1 | MS0011 | BV0011 | CJ0011 |
| Row 2 | MS0012 | BV0012 | CJ0012 |
| Row 3 | MS0013 | BV0013 | CJ0013 |

`MS`=Mureș, `BV`=Brașov, `CJ`=Cluj. The prefix (first 2 letters) = **destination/column**; the rest = the part/position ID.

### 6.2 Sorting strategy (pick one)
- **A. Fixed code→cell mapping:** each QR code has a predefined cell in a `lookup[code] = {x,y,z}` table. Simple, deterministic. Good if the number of codes is small and fixed.
- **B. Sort by column + first free cell:** the prefix picks the column (MS/BV/CJ), and the part goes into the first free cell of that column (keep an occupancy counter per column). More flexible, scales well. **Recommended** for the flow shown in the video.

For both, you need a **table of calibrated coordinates** for each shelf cell (X,Y of the cell center + placement Z) plus the **pick zone** (where boxes arrive) and the **scan position** (where you bring the camera to read the QR).

### 6.3 Data structures (on the brain)
```cpp
struct Pos { long x, y, z; };          // in absolute steps
Pos PICK_POS;                          // pick zone
Pos SCAN_POS;                          // QR read position
Pos SHELF[3][3];                       // shelf grid (calibrated)
const char* COL_PREFIX[3] = {"MS","BV","CJ"};
int colFill[3] = {0,0,0};              // occupied cells per column
```

---

## 7. Brain state machine

```
IDLE
  └─> HOME (home Z,X,Y)              → ORIGIN_SET
ORIGIN_SET
  └─> GOTO PICK_POS                  → AT_PICK
AT_PICK
  └─> Z down, close gripper (SG90)   → GRABBED
  └─> Z up
GRABBED
  └─> GOTO SCAN_POS                  → AT_SCAN
AT_SCAN
  └─> send trigger to CAM, wait for QR (timeout)
        ├─ QR received → DECIDE_TARGET
        └─ timeout/error → ERROR_BIN (reject bin) or re-scan
DECIDE_TARGET
  └─> prefix → column → first free cell → target = SHELF[r][c]
PLACE
  └─> GOTO target (X,Y), Z down, open gripper, Z up
  └─> colFill[c]++                   → CYCLE_DONE
CYCLE_DONE
  └─> more parts? → GOTO PICK_POS : IDLE
```

Important details:
- **Always raise Z fully** before any X/Y move (avoids collisions with the shelves).
- **Scan timeout** (e.g. 3 s): if no valid QR arrives, send the part to a "reject bin" and log it.
- **QR validation:** check the format (known 2-letter prefix + digits) before accepting. Ignore partial/corrupt strings.
- **Serial buffer:** read line by line up to `\n`; discard incomplete lines.

---

## 8. Serial protocol ESP32-CAM → brain

Simple ASCII protocol, newline-terminated. Robust and easy to debug on a serial monitor.

**CAM → brain (data):**
```
QR:CJ0012\n      // valid decoded code
NOQR\n           // (optional) no code in frame
```
**brain → CAM (control, optional on UART2 TX / GPIO14):**
```
SCAN\n           // trigger one capture + decode
```

Two operating modes — pick one:
- **Trigger-based (recommended):** the CAM waits; on `SCAN\n` it captures, decodes, and replies `QR:...` or `NOQR`. Deterministic, synchronized with the state machine.
- **Free-running:** the CAM scans continuously and sends `QR:...` whenever it sees a code. Simpler, but the brain must ignore codes when not in the `AT_SCAN` state.

Parameters: **115200 baud**, 8N1. Both boards are 3.3V → direct TX↔RX connection, **common GND**. If you want isolation, add a 1k series resistor on the line.

---

## 9. Phased implementation plan

Ordered to isolate risk — don't skip phases, each validates the assumptions of the next.

**Phase 0 — Prep & decisions (1–2 days)**
- Confirm each actuator's screw lead (→ steps/mm).
- Decide A4988 vs DRV8825 (§2.1) and the CAM power scheme (§2.2).
- Order the missing parts from §3 (heatsinks, capacitors, 5V buck, drag chain).

**Phase 1 — Mechanics**
- Mount the 3 actuators orthogonally (Y base → X bridge → Z vertical).
- Mount the gripper (GrabCAD) + SG90 + ESP32-CAM holder on the Z carriage.
- Install the endstops: one MIN/home switch at the "home" end of each axis, plus the 3 MAX switches (in series) at the far end of each axis (6 total).
- Route the cabling through the drag chain to the moving carriage.

**Phase 2 — Electronics on the bench (one axis)**
- Power ONLY one A4988 + one motor. **Tune Vref** (below). Verify the coil pairing.
- Test a single stepper from the brain (forward/backward rotation).
- A4988 Vref tuning: `Vref ≈ I_trip × 8 × Rsense`. For modules with Rsense=0.1Ω and a 1.0–1.2A target → `Vref ≈ 0.8–0.96 V`. Measure Vref between the potentiometer and GND, motor disconnected for the initial setting. **Confirm your board's actual Rsense** (could be 0.05/0.1/0.2Ω) — change the formula accordingly.

**Phase 3 — Brain firmware: one axis + homing**
- Integrate FastAccelStepper. Absolute moves in steps.
- Implement homing on one axis (approach/back-off/slow re-approach).
- Calibrate `steps_per_mm` empirically (command 100 mm, measure).

**Phase 4 — 3 axes + gripper + coordinates**
- Extend to X, Y, Z. Full homing (order Z→X→Y).
- Add SG90 on LEDC; test open/close.
- Define `PICK_POS`, `SCAN_POS` and the `SHELF[3][3]` grid by manual jogging + reading the position (calibration §10).
- Soft limits + raise Z before XY moves.

**Phase 5 — Standalone ESP32-CAM vision**
- Flash `ESP32QRCodeReader`. Test decoding a printed QR at the fixed scan distance. Adjust illumination/resolution until the rate is stable.
- Print the string on the serial monitor; confirm correct `MS/BV/CJ` reads.

**Phase 6 — Serial link CAM↔brain**
- Implement the §8 protocol (start free-running, then trigger).
- Confirm on the brain that it receives complete `QR:...`, no corrupt lines, with common GND.

**Phase 7 — Integration: full state machine**
- Wire the whole §7 flow: pick → scan → decide → place → repeat.
- Test with 2–3 boxes and different codes. Verify column sorting.
- Add error handling: scan timeout, reject bin, invalid QR.

**Phase 8 — Fine calibration & robustness**
- Tune the shelf coordinates until placement is repeatable (§10).
- Tune speeds/accelerations for fast cycles without lost steps.
- Endurance test: 50–100 continuous cycles, measure drift.

**Phase 9 — Finishing**
- Electronics enclosure, cable fixing, fuse on the 12V.
- Emergency stop button (cuts the drivers' EN + the motor power).
- (Optional) small improvements: status LEDs, serial/SD logging.

---

## 10. Calibration & tuning (checklist)

- **steps/mm per axis** — the 100 mm-measured-with-calipers method.
- **Vref per driver** — set to target current, with the heatsink mounted.
- **Reference positions** — `PICK_POS`, `SCAN_POS` and the 9 shelf cells, saved as absolute steps. Method: after homing, manually jog to each position and read the step counter.
- **Scan distance/focus** — fixed, validated with a read rate ≥90%.
- **Speed & acceleration** — raise gradually to the threshold, then back off 20% for margin.
- **Repeatability test** — return 10× to the same cell, measure the deviation (target < 0.5 mm).

---

## 11. Main risks & mitigations

| Risk | Cause | Mitigation |
|---|---|---|
| ESP32-CAM brownout/reset | Weak buck, camera inrush | Dedicated 5V buck + 470µF cap (§2.2) |
| A4988 overheating / lost steps | 1.7A above the comfortable threshold | Heatsink, reduce current to ~1.1A, or DRV8825 |
| Burned driver | VMOT spike with no capacitor | 100µF cap on each driver's VMOT |
| Poor QR read rate | Fixed focus, low light | Fixed distance, illumination, larger QR; fallback Tiny Code Reader |
| Collision with the shelf | XY move with Z down | Raise Z fully before any XY move |
| Corrupt UART / "doesn't work" | No common GND, GPIO12 strapping | Common GND, CAM TX on GPIO13 not 12 |
| Unreliable homing | Mechanical bounce, broken wire | Slow re-approach + debounce + NC fail-safe endstop |
| ESP32 boot stuck | Strapping pins used wrong | Avoid GPIO 0/2/12/15 for signals active at boot |

---

## 12. Next steps (after plan approval)

1. **ESP32-CAM code** — capture + QR decode + serial protocol (§8).
2. **Brain ESP32 code (WROOM-32D)** — FastAccelStepper, homing, state machine (§7), SG90, QR parsing, sorting logic (§6).
3. **EasyEDA schematic** — per pin map §4: the 3 A4988s, 12V distribution, the bucks, ESP32-CAM, the brain, the 6 endstops (3 MIN + 3 MAX in series on GPIO4), SG90, the protection capacitors.

Recommended to do the EasyEDA schematic + the brain code in parallel, since the §4 pin map ties both together.
