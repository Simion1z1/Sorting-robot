/* =============================================================================
   ESP32-CAM MJPEG stream  (Sorting Robot — PC-decode variant)
   -----------------------------------------------------------------------------
   This is the ALTERNATIVE pipeline: the camera does NO decoding. It just streams
   MJPEG over WiFi so a PC (Python + OpenCV/pyzbar) can decode the QR with much
   higher accuracy, then forward "QR:<code>" to the brain over USB serial.

   This is a SEPARATE sketch — esp32cam_qr_test.ino (on-board decode + ESP-NOW)
   stays untouched as the working fallback.

   PC side: open  http://<this-ip>/stream  in a browser to check it, or point
   pc/decode_qr.py at it.

   BOARD SETTINGS (Arduino IDE > Tools):
     Board: "AI Thinker ESP32-CAM"   PSRAM: Enabled
     Partition: "Huge APP (3MB No OTA/1MB SPIFFS)"
============================================================================= */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

// ----------------------------- CONFIG ---------------------------------------
const char *WIFI_SSID = "Tenda_E046F0";       // <-- fill in
const char *WIFI_PASS = "istefan158";   // <-- fill in

#define STREAM_FRAMESIZE  FRAMESIZE_SVGA   // SVGA 800x600 (more pixels on the QR). VGA = faster.
#define JPEG_QUALITY      10               // 10-12 good; lower number = better quality, bigger frames

#define USE_FLASH_LED     true             // steady illumination for the PC to see the QR
#define FLASH_LED_PIN     4
#define FLASH_LEVEL       0                // 0-255 steady brightness (0 = off). Tune for best PC decode.
#define FLASH_LEDC_CH     7                // camera uses LEDC ch0/timer0 — keep this clear
// ----------------------------------------------------------------------------

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

httpd_handle_t stream_httpd = NULL;

void flashSetup() {
#if USE_FLASH_LED
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(FLASH_LED_PIN, 5000, 8);
    ledcWrite(FLASH_LED_PIN, FLASH_LEVEL);
  #else
    ledcSetup(FLASH_LEDC_CH, 5000, 8);
    ledcAttachPin(FLASH_LED_PIN, FLASH_LEDC_CH);
    ledcWrite(FLASH_LEDC_CH, FLASH_LEVEL);
  #endif
#endif
}

static esp_err_t stream_handler(httpd_req_t *req) {
  static const char *CT  = "multipart/x-mixed-replace;boundary=frame";
  static const char *BND = "\r\n--frame\r\n";
  static const char *HDR = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
  char part[64];
  esp_err_t res = httpd_resp_set_type(req, CT);
  if (res != ESP_OK) return res;

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    res = httpd_resp_send_chunk(req, BND, strlen(BND));
    if (res == ESP_OK) {
      size_t n = snprintf(part, sizeof(part), HDR, fb->len);
      res = httpd_resp_send_chunk(req, part, n);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;       // client disconnected
  }
  return res;
}

void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  httpd_uri_t stream_uri = { "/stream", HTTP_GET, stream_handler, NULL };
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("Stream server started at /stream");
  } else {
    Serial.println("httpd_start FAILED");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32-CAM MJPEG stream (PC-decode variant) ===");

  flashSetup();

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = STREAM_FRAMESIZE;
  config.jpeg_quality = JPEG_QUALITY;
  config.fb_count     = psramFound() ? 2 : 1;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init FAILED");
    delay(2000);
    ESP.restart();
  }

  // a little contrast helps the PC decoder too
  sensor_t *s = esp_camera_sensor_get();
  if (s) { s->set_contrast(s, 1); s->set_hmirror(s, 0); s->set_vflip(s, 0); }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.printf("\nWiFi OK. Stream: http://%s/stream\n", WiFi.localIP().toString().c_str());

  startServer();
}

void loop() {
  delay(1000);   // everything runs in the http server task
}
