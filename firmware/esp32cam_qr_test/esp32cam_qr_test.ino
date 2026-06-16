/* =============================================================================
   Step 1 — ESP32-CAM QR test  (Sorting Robot)
   -----------------------------------------------------------------------------
   Goal of this sketch (Phase 1 / Stage 1 of the implementation plan):
     - Recognize a printed QR code with the ESP32-CAM (AI-Thinker).
     - Print what it sees on the USB serial monitor (debug).
     - Emit the "order" to the brain over a separate UART, in the §8 format:
           QR:CJ0012\n
       so the pipeline  "see QR  ->  send order"  is proven before you build
       the rest of the machine.

   This runs STANDALONE. You do not need the brain ESP32 connected to test it —
   just watch the USB serial monitor, and optionally tap GPIO13 with a USB-TTL
   adapter to see the order line that will go to the brain.

   LIBRARY (install once):
     ESP32QRCodeReader by alvarowolfx
     https://github.com/alvarowolfx/ESP32QRCodeReader
     Arduino IDE: Sketch > Include Library > Add .ZIP Library...  (download ZIP)

   BOARD SETTINGS (Arduino IDE > Tools):
     Board:          "AI Thinker ESP32-CAM"
     PSRAM:          Enabled
     Partition:      "Huge APP (3MB No OTA/1MB SPIFFS)"
     Upload Speed:   115200 (or 921600)
   Program it on the HW-381 / ESP32-CAM-MB carrier (USB). UART0 (GPIO1/3) is the
   USB debug port; the brain link below uses a SEPARATE UART on GPIO13/14.
============================================================================= */

#include "ESP32QRCodeReader.h"

// ----------------------------- CONFIG ---------------------------------------
#define BRAIN_BAUD        115200   // UART to the brain (and 8N1)
#define BRAIN_TX_PIN      13       // CAM -> brain  (QR data)   [do NOT use 12]
#define BRAIN_RX_PIN      14       // brain -> CAM  (optional SCAN trigger)

#define USE_TRIGGER       false    // false = free-running (sends when it sees a code)
                                   // true  = wait for "SCAN\n" from the brain, then send once
#define SEND_COOLDOWN_MS  1500     // don't resend the SAME code faster than this (anti-spam)

#define USE_FLASH_LED     false    // GPIO4 white LED as illumination (VERY bright/hot — use briefly)
#define FLASH_LED_PIN     4
#define STATUS_LED_PIN    33       // onboard red LED (active LOW) — blinks on a read
// ----------------------------------------------------------------------------

// Framesize MUST be set here (constructor): esp_camera_init() allocates the
// frame buffer for THIS size. Changing it later via set_framesize() to anything
// larger gives no valid frames. SVGA (800x600) is the library's max (it refuses
// anything > FRAMESIZE_SVGA) and matches the resolution you verified clear in the
// webserver. Drop to FRAMESIZE_VGA if you want faster decodes / less memory.
ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER, FRAMESIZE_SVGA);
HardwareSerial    Brain(1);        // UART1 -> brain

String        lastCode   = "";
unsigned long lastSendMs = 0;

#if USE_TRIGGER
  volatile bool gArmed = false;    // becomes true when "SCAN" arrives
#else
  volatile bool gArmed = true;     // always ready in free-running mode
#endif

// Optional sanity check of the expected code shape: 2-letter prefix + digits.
// We still FORWARD whatever we read; the brain makes the final decision.
bool isValidCode(const String &s) {
  if (s.length() < 3) return false;
  String p = s.substring(0, 2);
  if (!(p == "MS" || p == "BV" || p == "CJ")) return false;
  for (size_t i = 2; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

void sendOrder(const String &code) {
  Brain.print("QR:");
  Brain.print(code);
  Brain.print('\n');                       // newline-terminated, per §8
  Serial.print("[SENT -> brain]  QR:");
  Serial.println(code);
}

void onQrCodeTask(void *pv) {
  struct QRCodeData qrCodeData;
  while (true) {
    if (reader.receiveQrCode(&qrCodeData, 100)) {
      if (qrCodeData.valid) {
        String code = String((const char *)qrCodeData.payload);
        code.trim();

        Serial.print("[QR] payload=\"");
        Serial.print(code);
        Serial.print("\"  format=");
        Serial.println(isValidCode(code) ? "OK (MS/BV/CJ + digits)" : "unexpected");

        unsigned long now = millis();
        bool fresh = (code != lastCode) || (now - lastSendMs > SEND_COOLDOWN_MS);

        if (gArmed && fresh) {
          digitalWrite(STATUS_LED_PIN, LOW);   // red ON (active low)
          sendOrder(code);
          lastCode   = code;
          lastSendMs = now;
        #if USE_TRIGGER
          gArmed = false;                       // one reply per SCAN
        #endif
          delay(60);
          digitalWrite(STATUS_LED_PIN, HIGH);   // red OFF
        }
      } else {
        Serial.println("[QR] code detected but not decodable (blurry / low light / too small)");
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// Apply the same camera settings that gave a sharp image in the webserver.
// NOTE: the QR reader decodes GRAYSCALE frames, so JPEG "Quality" is irrelevant
// here — what matters for quirc is RESOLUTION (more pixels on the QR) + contrast.
// Must be called AFTER reader.setup() (camera must already be initialised).
void applyCameraTuning() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("[CAM] sensor_get failed — tuning skipped");
    return;
  }
  // NOTE: framesize is set in the constructor (SVGA), NOT here — raising it at
  // runtime above the init size kills the frame stream. Only image controls below.

  // Match the webserver panel (centred sliders = 0):
  s->set_brightness(s, 0);
  s->set_contrast(s, 1);          // +1 crisps the black/white edges for quirc
  s->set_saturation(s, 0);        // ignored in grayscale, harmless
  s->set_whitebal(s, 1);          // AWB on
  s->set_awb_gain(s, 1);          // AWB gain on
  s->set_wb_mode(s, 0);           // WB mode = Auto
  s->set_exposure_ctrl(s, 1);     // AEC sensor on
  s->set_aec2(s, 0);              // AEC DSP off
  s->set_ae_level(s, 0);
  s->set_gain_ctrl(s, 1);         // AGC on
  s->set_gainceiling(s, GAINCEILING_2X);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);              // lens correction on
  s->set_dcw(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);

  Serial.println("[CAM] tuning applied: SVGA, AWB/AEC/AGC on, contrast +1");
}

void setup() {
  Serial.begin(115200);                                        // USB debug (UART0)
  Brain.begin(BRAIN_BAUD, SERIAL_8N1, BRAIN_RX_PIN, BRAIN_TX_PIN); // link to brain (UART1)

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);                          // off
#if USE_FLASH_LED
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, HIGH);                           // illumination on
#endif

  Serial.println();
  Serial.println("=== ESP32-CAM QR test (Sorting Robot, Step 1) ===");
  Serial.print  ("Mode: ");
  Serial.println(USE_TRIGGER ? "TRIGGER (waiting for SCAN\\n)" : "FREE-RUNNING");

  reader.setup();
  applyCameraTuning();                                         // <-- SVGA + webserver settings
  Serial.println("Camera initialised. Hold a printed QR ~10-20 cm away.");
  reader.beginOnCore(1);

  xTaskCreate(onQrCodeTask, "onQrCode", 4096, NULL, 4, NULL);
}

void loop() {
#if USE_TRIGGER
  // Listen for the brain's "SCAN\n" command to arm a single read.
  static String rx;
  while (Brain.available()) {
    char c = Brain.read();
    if (c == '\n') {
      rx.trim();
      if (rx == "SCAN") {
        gArmed = true;
        Serial.println("[TRIG] SCAN received -> armed");
      }
      rx = "";
    } else if (rx.length() < 32) {
      rx += c;
    }
  }
#endif
  delay(10);
}
