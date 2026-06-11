# Schematic netlist — Sorting Robot (for EasyEDA recreation)

Follow this net-by-net to rebuild the schematic in EasyEDA. It matches the pin map in `IMPLEMENTATION_PLAN.md` §4. Place each component, then connect every net listed. Anything on the **same net name** is wired together.

---

## 1. Component list (designators)

| Ref | Part | Notes |
|---|---|---|
| PSU1 | 12V DC power supply | ≥6A recommended |
| F1 | Fuse 5A | in series with 12V+ |
| BK1 | Buck converter 12V→3.3V | powers the brain logic |
| BK2 | Buck converter 12V→5V | powers ESP32-CAM + servo |
| U1 | ESP32-DevKitC WROOM-32D | the brain |
| U2 | ESP32-CAM (AI-Thinker) | vision node |
| A1 / A2 / A3 | A4988 driver | X / Y / Z |
| M1 / M2 / M3 | NEMA 17 stepper | X / Y / Z (inside actuators) |
| SW1 / SW2 / SW3 | SS-5GL endstop | X / Y / Z home switches |
| SV1 | SG90 servo | gripper |
| C1 / C2 / C3 | 100 µF / 35V electrolytic | one across each A4988 VMOT–GND |
| C4 | 470–1000 µF electrolytic | across ESP32-CAM 5V–GND |
| C5 (opt) | 100 nF ceramic | across CAM 3V3–GND for noise |

---

## 2. Power nets

**Net `+12V`** (thick trace):
`PSU1(+)` → `F1` → `A1.VMOT`, `A2.VMOT`, `A3.VMOT`, `BK1.IN+`, `BK2.IN+`

**Net `+3V3`** (from BK1):
`BK1.OUT+` → `U1.3V3`, `A1.VDD`, `A2.VDD`, `A3.VDD`
(also feeds the RESET–SLEEP tie and the MS jumpers — see §4)

**Net `+5V`** (from BK2):
`BK2.OUT+` → `U2.5V`, `SV1.VCC (red wire)`

**Net `GND`** (one common ground — CRITICAL, tie everything):
`PSU1(–)`, `F1` return side, `BK1.IN–`, `BK1.OUT–`, `BK2.IN–`, `BK2.OUT–`,
`U1.GND`, `U2.GND`, `A1.GND (both)`, `A2.GND (both)`, `A3.GND (both)`,
`M1/M2/M3` coil-common is NOT grounded (coils float), `SW1/2/3.COM`, `SV1.GND (brown)`,
`C1–C5` negative legs.

> The single most common failure is a missing common GND between the two ESP32s and the bucks. Make `GND` one continuous net.

**Decoupling caps:**
- `C1` across `A1.VMOT`–`GND`; `C2` across `A2.VMOT`–`GND`; `C3` across `A3.VMOT`–`GND`. Place physically next to each driver.
- `C4` across `U2.5V`–`GND`, right at the CAM power pins.

---

## 3. Signal nets — brain U1 → peripherals

### Stepper STEP/DIR (one net each)
| Net | From (U1) | To |
|---|---|---|
| X_STEP | GPIO25 | A1.STEP |
| X_DIR  | GPIO26 | A1.DIR |
| Y_STEP | GPIO32 | A2.STEP |
| Y_DIR  | GPIO33 | A2.DIR |
| Z_STEP | GPIO27 | A3.STEP |
| Z_DIR  | GPIO14 | A3.DIR |

### Shared driver enable
| Net | From (U1) | To |
|---|---|---|
| EN | GPIO13 | A1.ENABLE, A2.ENABLE, A3.ENABLE (all three) |

> ENABLE is active-LOW. Optionally add a 10k pull-up from EN to 3V3 so the drivers stay disabled during boot.

### Endstops (SS-5GL, NC wiring, INPUT_PULLUP)
| Net | From (U1) | Switch | Switch other side |
|---|---|---|---|
| ES_X | GPIO21 | SW1.NC | SW1.COM → GND |
| ES_Y | GPIO22 | SW2.NC | SW2.COM → GND |
| ES_Z | GPIO23 | SW3.NC | SW3.COM → GND |

> Use the COM and NC terminals of the SS-5GL (leave NO unused). With `INPUT_PULLUP`: closed switch = LOW, triggered/broken wire = HIGH (fail-safe).

### Servo
| Net | From (U1) | To |
|---|---|---|
| SERVO_PWM | GPIO19 | SV1.signal (orange wire) |

> SV1 power is `+5V`/`GND` (§2), NOT 3.3V. The 3.3V GPIO19 signal drives the SG90 fine.

### UART link brain ↔ CAM (3.3V both sides, no level shifter)
| Net | From | To |
|---|---|---|
| QR_RX | U2.GPIO13 (CAM TX) | U1.GPIO16 (Serial2 RX) |
| SCAN_TX (optional) | U1.GPIO17 (Serial2 TX) | U2.GPIO14 (CAM trigger in) |

> Direction that matters is CAM→brain (`QR_RX`). Do NOT use CAM GPIO12 for TX (strapping). Add an optional 1k series resistor in each UART line if you want protection.

---

## 4. Per-A4988 wiring (repeat for A1, A2, A3)

```
VMOT      → +12V          (+ C across VMOT–GND, placed close)
GND(mot)  → GND
VDD       → +3V3
GND(log)  → GND
STEP      → U1 GPIO (table §3)
DIR       → U1 GPIO (table §3)
ENABLE    → EN net (shared GPIO13)
RESET     ─┐ tie together, then → +3V3
SLEEP     ─┘
MS1,MS2,MS3 → jumper to +3V3 for 1/16 microstepping (all HIGH)
1A,1B     → motor coil A   (verify the pair with an ohmmeter!)
2A,2B     → motor coil B
```

Coil pairing for NEMA17: the two wires of one coil read a few ohms across each other and **open** to the other coil. Get this wrong and the motor just vibrates instead of turning. The four motor wires go to `1A,1B,2A,2B`; keep each coil's two wires together on the A/B pair.

Microstepping table (set with the 3 jumpers): for 1/16, tie MS1=MS2=MS3 to +3V3. (1/8 = MS1,MS2 high, MS3 low; full step = all low.)

---

## 5. ESP32-CAM (U2) connections

```
5V        → +5V        (+ C4 470µF across 5V–GND)
GND       → GND
GPIO13    → QR_RX net   (→ U1 GPIO16)   [CAM transmits decoded QR]
GPIO14    → SCAN_TX net (← U1 GPIO17)   [optional scan trigger in]
GPIO4     → onboard white LED (optional illumination, leave as-is)
U0R/U0T   → only for flashing via the HW-381 / ESP32-CAM-MB board
```

> Program U2 separately on the HW-381 (ESP32-CAM-MB) carrier, then move it onto the machine. Do not power the CAM from both 5V and 3V3 at once.

---

## 6. Brain U1 (ESP32 WROOM-32D) pin summary

| Pin | Net |
|---|---|
| 3V3 | +3V3 (from BK1) |
| GND | GND |
| GPIO25 | X_STEP |
| GPIO26 | X_DIR |
| GPIO32 | Y_STEP |
| GPIO33 | Y_DIR |
| GPIO27 | Z_STEP |
| GPIO14 | Z_DIR |
| GPIO13 | EN (shared) |
| GPIO21 | ES_X |
| GPIO22 | ES_Y |
| GPIO23 | ES_Z |
| GPIO19 | SERVO_PWM |
| GPIO16 | QR_RX (Serial2 RX) |
| GPIO17 | SCAN_TX (Serial2 TX, optional) |
| (GPIO2) | optional status LED |

Avoid for signals: GPIO 0/2/12/15 (strapping), 6–11 (flash), 34–39 (input-only).

---

## 7. EasyEDA build order (suggested)

1. Place U1, U2, A1–A3, BK1, BK2, PSU1/F1, SW1–3, SV1, M1–M3, caps.
2. Wire the three power nets (`+12V`, `+3V3`, `+5V`) and the single `GND` net first — use net labels, not long wires, to keep it readable.
3. Add the STEP/DIR/EN nets, then endstops, servo, UART.
4. Drop the decoupling caps next to their components.
5. Run **DRC**. Fix any unconnected pins. Export the schematic PDF.

After the schematic passes DRC, you can move to PCB or just use it as the wiring reference for a protoboard build.
