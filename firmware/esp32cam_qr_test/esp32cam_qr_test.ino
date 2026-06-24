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
   Program it on the HW-381 / ESP32-CAM-MB carrier (USB).

   POWER: powered over USB via the ESP32-CAM-MB (HW-381) carrier — from a laptop USB
   or a power bank (5V). Clean 5V, no buck. The brain link is WIRELESS (ESP-NOW), so the
   camera needs only power: no data wire and no common GND with the brain. (Use a power
   bank without auto-off, or it may switch off at low idle current.)
============================================================================= */

#include "ESP32QRCodeReader.h"
#include <WiFi.h>
#include <esp_now.h>

// ----------------------------- CONFIG ---------------------------------------
#define BRAIN_BAUD        9600     // UART to the brain (8N1). Low baud = robust over marginal wiring.
#define BRAIN_TX_PIN      13       // CAM -> brain  (QR data)   [do NOT use 12]
#define BRAIN_RX_PIN      14       // brain -> CAM  (optional SCAN trigger)

#define USE_TRIGGER       false    // false = free-running (sends when it sees a code)
                                   // true  = wait for "SCAN\n" from the brain, then send once
#define SEND_COOLDOWN_MS  1500     // don't resend the SAME code faster than this (anti-spam)

#define USE_FLASH_LED     true     // master enable for the GPIO4 white LED
#define FLASH_LED_PIN     4
#define FLASH_LEVEL       50        // 0-255 STEADY illumination. 0 = off (only blinks on scan); >0 = constant light
#define FLASH_LEDC_CH     7        // LEDC channel (camera uses ch0/timer0 — keep this one clear)
#define STATUS_LED_PIN    33       // onboard red LED (active LOW) — blinks on a read
// ----------------------------------------------------------------------------

// Framesize MUST be set here (constructor): esp_camera_init() allocates the
// frame buffer for THIS size. Changing it later via set_framesize() to anything
// larger gives no valid frames. SVGA (800x600) is the library's max (it refuses
// anything > FRAMESIZE_SVGA) and matches the resolution you verified clear in the
// webserver. Drop to FRAMESIZE_VGA if you want faster decodes / less memory.
ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER, FRAMESIZE_SVGA);   // SVGA 800x600 = more pixels on the code (library max); FRAMESIZE_VGA for faster decodes
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
  if (!(p == "MS" || p == "BV" || p == "CJ" || p == "EB")) return false;
  for (size_t i = 2; i < s.length(); i++) {
    if (!isDigit(s[i])) return false;
  }
  return true;
}

// ESP-NOW broadcast: send the order wirelessly to the brain (no wire needed).
uint8_t broadcastAddr[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

void initEspNow() {
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init FAILED"); return; }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddr, 6);
  peer.channel = 0;        // use the current WiFi channel (both boards default to ch1)
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) { Serial.println("ESP-NOW add_peer FAILED"); return; }
  Serial.printf("ESP-NOW ready (CAM). MAC = %s\n", WiFi.macAddress().c_str());
}

void sendOrder(const String &code) {
  String msg = "QR:" + code;                                   // same §8 format, over ESP-NOW
  esp_now_send(broadcastAddr, (const uint8_t *)msg.c_str(), msg.length());
  Serial.print("[SENT -> brain]  ");
  Serial.println(msg);
}

// White GPIO4 LED via PWM: FLASH_LEVEL = steady illumination (0..255), and a blink
// pulses to full brightness then returns to the steady level.
#if USE_FLASH_LED
void flashWrite(uint8_t level) {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(FLASH_LED_PIN, level);
  #else
    ledcWrite(FLASH_LEDC_CH, level);
  #endif
}
void flashSetup() {
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(FLASH_LED_PIN, 5000, 8);
  #else
    ledcSetup(FLASH_LEDC_CH, 5000, 8);
    ledcAttachPin(FLASH_LED_PIN, FLASH_LEDC_CH);
  #endif
  flashWrite(FLASH_LEVEL);                 // steady level (0 = off)
}
void blinkFlash(int times) {
  for (int i = 0; i < times; i++) {
    flashWrite(255);                        // pulse to full
    delay(60);
    flashWrite(FLASH_LEVEL);                // back to steady level
    if (i < times - 1) delay(80);
  }
}
#else
void flashSetup() {}
void blinkFlash(int) {}
#endif

void onQrCodeTask(void *pv) {
  struct QRCodeData qrCodeData;
  unsigned long nDecoded = 0, nDetectedBad = 0;   // read-rate stats
  while (true) {
    if (reader.receiveQrCode(&qrCodeData, 100)) {
      if (qrCodeData.valid) {
        nDecoded++;
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
          blinkFlash(2);                        // white blinks TWICE on a good scan
          digitalWrite(STATUS_LED_PIN, HIGH);   // red OFF
        }
      } else {
        nDetectedBad++;
        Serial.println("[QR] code detected but not decodable (blurry / low light / too small)");
      }
      // Live read-rate: decoded / (decoded + detected-but-bad). Aim for >=90%.
      unsigned long total = nDecoded + nDetectedBad;
      if (total > 0) {
        Serial.printf("[RATE] %lu/%lu decoded = %lu%%\n",
                      nDecoded, total, (nDecoded * 100) / total);
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
  s->set_contrast(s, 2);          // +2 hardens the black/white edges for quirc (was +1)
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
  flashSetup();                           // PWM on GPIO4; steady = FLASH_LEVEL (0 = off)
#endif

  Serial.println();
  Serial.println("=== ESP32-CAM QR test (Sorting Robot, Step 1) ===");
  Serial.print  ("Mode: ");
  Serial.println(USE_TRIGGER ? "TRIGGER (waiting for SCAN\\n)" : "FREE-RUNNING");

  reader.setup();
  applyCameraTuning();                                         // <-- SVGA + webserver settings
  Serial.println("Camera initialised. Hold a printed QR ~10-20 cm away.");
  reader.beginOnCore(1);

  initEspNow();                                                // wireless link to the brain

  xTaskCreate(onQrCodeTask, "onQrCode", 4096, NULL, 4, NULL);

  blinkFlash(2);                                               // boot OK -> two blinks
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
