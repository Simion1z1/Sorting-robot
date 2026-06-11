# Step 1 — ESP32-CAM QR test

Proves the vision pipeline before you build the machine: the camera reads a QR
and emits the order to the brain as `QR:CJ0012\n` (§8 protocol).

## 1. Install
- **esp32 core** (Espressif) via Boards Manager.
- **ESP32QRCodeReader** by alvarowolfx — download the ZIP from
  https://github.com/alvarowolfx/ESP32QRCodeReader and add it via
  *Sketch → Include Library → Add .ZIP Library…*

## 2. Board settings (Tools menu)
| Setting | Value |
|---|---|
| Board | AI Thinker ESP32-CAM |
| PSRAM | Enabled |
| Partition Scheme | Huge APP (3MB No OTA) |
| Upload Speed | 115200 (or 921600) |

Flash it on the **HW-381 / ESP32-CAM-MB** carrier over USB.

## 3. How to test
1. Open Serial Monitor at **115200** (this is the USB debug log on UART0).
2. Hold a printed QR ~10–20 cm in front of the lens, good lighting.
3. You should see:
   ```
   [QR] payload="CJ0012"  format=OK (MS/BV/CJ + digits)
   [SENT -> brain]  QR:CJ0012
   ```
4. To watch the **actual line going to the brain**, connect a USB-TTL adapter:
   - CAM **GPIO13** → adapter **RX**
   - CAM **GND** → adapter **GND**
   - Open that adapter at 115200 — you'll see `QR:CJ0012` lines.
   (This is the same wire that later goes to the brain's GPIO16.)

## 4. Pass/fail gate (from the plan)
Tune lighting + distance until the read rate is **≥90%** over ~20 tries.
If you can't get there with the ESP32-CAM, this is the moment to switch to the
**Tiny Code Reader (I2C)** or **GM65 (UART)** fallback — the protocol stays the
same, only the source of the string changes.

## 5. Options (top of the .ino)
- `USE_TRIGGER` — `false` = free-running (sends whenever it sees a code);
  `true` = waits for `SCAN\n` from the brain, then replies once. Use `true`
  later when integrating with the state machine.
- `SEND_COOLDOWN_MS` — anti-spam; won't resend the same code faster than this.
- `USE_FLASH_LED` — pulse the bright GPIO4 LED for illumination (off by default).

## 6. Note on the vision self-correction (later)
This test forwards the **code only**. The `DX/DY/TH` pixel-offset fields from
§6.4 need the QR **corner coordinates**, which the simple library hides. We'll
add those in Step 7 by reading quirc's corner points directly — not needed yet.
