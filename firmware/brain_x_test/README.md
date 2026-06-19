# Brain firmware — X-axis bench test

First brain (ESP32 WROOM-32D) firmware. Scope: drive **only the X stepper**
through its A4988 with FastAccelStepper. **No homing, no endstops, no Y/Z.**
Maps to Plan §9 Stage 5 / step 28 — "a clean move on one axis".

## 1. Install
- **esp32 core** (Espressif) via Boards Manager.
- **FastAccelStepper** by *gin66* — *Tools → Manage Libraries…* → search & install.

## 2. Board settings (Tools menu)
| Setting | Value |
|---|---|
| Board | ESP32 Dev Module |
| Upload Speed | 115200 (or 921600) |

## 3. Wiring (per §4.2 / §4.4)
| A4988 (X) | Brain GPIO |
|---|---|
| STEP | 25 |
| DIR | 26 |
| ENABLE | 13 (active LOW) |
| VDD / logic-GND | 3.3V / GND |
| VMOT / motor-GND | 12V (+100µF cap) |
| RESET–SLEEP | tied together → VDD |
| 1A,1B,2A,2B | motor coils (verify pairs with ohmmeter!) |
| MS1/MS2/MS3 | jumpers (all HIGH = 1/16) |

> **Set Vref first** (motor disconnected) — see §9 Stage 4. An untuned A4988
> means lost steps or a burned driver.

## 4. Serial commands (115200, send a newline)
| Cmd | Action |
|---|---|
| `M<mm>` | relative move (`M50`, `M-20.5`) |
| `G<mm>` | absolute move (`G0`, `G100`) |
| `V<mm/s>` | set max speed |
| `A<mm/s2>` | set acceleration |
| `Z` | zero current position |
| `P` | print position + settings |
| `X` | stop now |
| `N1` / `N0` | enable / disable driver (N0 = turn screw by hand) |
| `C<mm>` | calibration helper (below) |
| `?` | help |

## 5. First run
1. `N0` then turn the screw by hand to a safe mid-travel spot. `N1`, then `Z`.
2. Start tiny: `V5`, `A100`, then `M10`. Confirm direction and smooth motion.
   - Wrong direction? Swap the X DIR logic later, or flip one coil pair.
   - Driver hot / stalling? Re-check Vref and microstepping jumpers.

## 6. Calibrate steps/mm (§5.2)
`steps_per_mm` defaults to **800** (200 steps × 1/16 ÷ 4 mm lead) — **almost
certainly wrong for your actuator.** Fix it empirically:
1. `Z` to zero, then `C100` (commands a 100 mm move).
2. Measure the **actual** travel with calipers.
3. Send just that measured number (e.g. `98.7`). The sketch recomputes
   `steps_per_mm = steps_per_mm × (commanded / measured)`.
4. Repeat `C100` → measure → confirm error < 0.2 mm.

> The corrected value lives in RAM only. Once happy, copy it into
> `float steps_per_mm = ...;` at the top of the `.ino` so it survives reboot.

## 7. Next
Once X moves cleanly and steps/mm is calibrated: add Y and Z (same pattern,
pins from §4.2), then homing once the endstops are wired (Stage 5, step 29+).
