/* =============================================================================
   ESP32-CAM — LOCAL QR test (no brain, no wiring)
   -----------------------------------------------------------------------------
   Purpose: confirm the camera works and decodes QR codes, using ONLY the USB
   cable and the Arduino Serial Monitor. Nothing else needs to be connected.

   What you'll see when a QR is decoded:
       #1  [QR OK ]  "CJ0012"   (len 6)   reads=1  misses=0
   "misses" counts frames where a code was seen but couldn't be decoded
   (blurry / too small / poor light) — use the ratio to judge the ~90% gate.

   LIBRARY:  ESP32QRCodeReader by alvarowolfx
             https://github.com/alvarowolfx/ESP32QRCodeReader
             (Sketch > Include Library > Add .ZIP Library...)

   BOARD (Tools menu):
       Board:      AI Thinker ESP32-CAM
       PSRAM:      Enabled
       Partition:  Huge APP (3MB No OTA)
   Flash on the HW-381 / ESP32-CAM-MB carrier, then open Serial Monitor @115200.
============================================================================= */

#include "ESP32QRCodeReader.h"

#define STATUS_LED_PIN   33      // onboard red LED (active LOW) — blinks on a good read
#define USE_FLASH_LED    false   // set true to light the bright GPIO4 LED for illumination
#define FLASH_LED_PIN    4

ESP32QRCodeReader reader(CAMERA_MODEL_AI_THINKER);

unsigned long reads   = 0;   // successful decodes
unsigned long misses  = 0;   // code seen but not decodable

void onQrCodeTask(void *pv) {
  struct QRCodeData qrCodeData;
  while (true) {
    if (reader.receiveQrCode(&qrCodeData, 100)) {
      if (qrCodeData.valid) {
        reads++;
        String code = String((const char *)qrCodeData.payload);
        code.trim();

        digitalWrite(STATUS_LED_PIN, LOW);       // red ON

        Serial.print("#");      Serial.print(reads);
        Serial.print("  [QR OK ]  \""); Serial.print(code);
        Serial.print("\"   (len "); Serial.print(code.length());
        Serial.print(")   reads="); Serial.print(reads);
        Serial.print("  misses="); Serial.println(misses);

        delay(80);
        digitalWrite(STATUS_LED_PIN, HIGH);      // red OFF
      } else {
        misses++;
        Serial.print("    [no read]  blurry/low-light/too-small   misses=");
        Serial.println(misses);
      }
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);            // off
#if USE_FLASH_LED
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, HIGH);             // illumination on
#endif

  Serial.println();
  Serial.println("=== ESP32-CAM LOCAL QR test ===");
  Serial.println("Hold a printed QR ~10-20 cm from the lens, well lit.");

  reader.setup();
  reader.beginOnCore(1);
  Serial.println("Camera ready. Watching for QR codes...");

  xTaskCreate(onQrCodeTask, "onQrCode", 4096, NULL, 4, NULL);
}

void loop() {
  delay(10);
}
