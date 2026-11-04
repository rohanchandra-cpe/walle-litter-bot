// Module 8 camera vision lab: ESP32-CAM photo server for the ROS 2 camera node.
//
// Joins your Wi-Fi and serves a fresh JPEG photo over HTTP:
//   http://CAMERA_IP/capture           a new photo (640 x 480 JPEG)
//   http://CAMERA_IP/capture?flash=1   the same, with the white flash LED on
//   http://CAMERA_IP/status            camera model, image size, Wi-Fi signal (JSON)
//   http://CAMERA_IP/                  a preview page for checking aim and focus
//
// Board: AI Thinker ESP32-CAM (OV3660 or OV2640 camera), on the ESP32-CAM-MB
// USB-C programmer. Build in VS Code with the PlatformIO extension (platformio.ini in
// this folder), or in the Arduino IDE with the board "AI Thinker ESP32-CAM".

#include <Arduino.h>
#include <WiFi.h>

#include "esp_camera.h"
#include "esp_http_server.h"

#include "wifi_config.h"   // WIFI_SSID and WIFI_PASSWORD (copy wifi_config.example.h)

// ===== Image orientation: change these if the preview is upside down or mirrored =====
#define CAMERA_VFLIP 1     // 1 = flip vertically
#define CAMERA_HMIRROR 0   // 1 = mirror left-right

// ===== AI Thinker ESP32-CAM pins =====
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
#define FLASH_LED_GPIO_NUM 4

static httpd_handle_t server = NULL;
static uint32_t photos_served = 0;

static bool start_camera() {
    camera_config_t config = {};
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    // 640 x 480 is plenty for a vision model and keeps each photo small (20-40 KB).
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;              // 0-63, lower = better quality, bigger file
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;

    if (!psramFound()) {
        Serial.println("No PSRAM found: check the board setting (AI Thinker ESP32-CAM).");
        config.frame_size = FRAMESIZE_QVGA;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x. Check the ribbon cable.\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        // Recommended starting settings for the OV3660 (from Espressif's example)
        s->set_brightness(s, 1);
        s->set_saturation(s, -2);
        Serial.println("Camera ready: OV3660, 640 x 480 JPEG.");
    } else {
        Serial.printf("Camera ready: sensor ID 0x%x, 640 x 480 JPEG.\n", s->id.PID);
    }
    s->set_vflip(s, CAMERA_VFLIP);
    s->set_hmirror(s, CAMERA_HMIRROR);
    return true;
}

// Returns a photo taken AFTER this call started, not one waiting in the buffer.
static camera_fb_t *take_fresh_photo() {
    camera_fb_t *old = esp_camera_fb_get();
    if (old) {
        esp_camera_fb_return(old);
    }
    return esp_camera_fb_get();
}

static esp_err_t capture_handler(httpd_req_t *req) {
    bool flash = false;
    char query[32];
    char value[8];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "flash", value, sizeof(value)) == ESP_OK) {
        flash = atoi(value) == 1;
    }
    if (flash) {
        digitalWrite(FLASH_LED_GPIO_NUM, HIGH);
        delay(150);   // let the auto exposure adjust to the light
    }
    camera_fb_t *fb = take_fresh_photo();
    if (flash) {
        digitalWrite(FLASH_LED_GPIO_NUM, LOW);
    }
    if (!fb) {
        Serial.println("Photo failed.");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t result = httpd_resp_send(req, (const char *) fb->buf, fb->len);
    esp_camera_fb_return(fb);
    photos_served++;
    return result;
}

static esp_err_t status_handler(httpd_req_t *req) {
    sensor_t *s = esp_camera_sensor_get();
    char json[160];
    snprintf(json, sizeof(json),
             "{\"sensor_id\":\"0x%x\",\"width\":640,\"height\":480,"
             "\"rssi_dbm\":%d,\"photos_served\":%lu}",
             s ? s->id.PID : 0, WiFi.RSSI(), (unsigned long) photos_served);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t index_handler(httpd_req_t *req) {
    static const char page[] =
        "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width'>"
        "<title>ESP32-CAM</title></head><body style='font-family:sans-serif'>"
        "<h2>ESP32-CAM preview</h2>"
        "<p><button onclick=\"img.src='/capture?t='+Date.now()\">New photo</button> "
        "<button onclick=\"img.src='/capture?flash=1&t='+Date.now()\">New photo with flash</button></p>"
        "<img id='img' src='/capture' style='max-width:100%'></body></html>";
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, page);
}

static void start_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    if (httpd_start(&server, &config) != ESP_OK) {
        Serial.println("HTTP server failed to start.");
        return;
    }
    httpd_uri_t index_uri = {"/", HTTP_GET, index_handler, NULL};
    httpd_uri_t capture_uri = {"/capture", HTTP_GET, capture_handler, NULL};
    httpd_uri_t status_uri = {"/status", HTTP_GET, status_handler, NULL};
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &capture_uri);
    httpd_register_uri_handler(server, &status_uri);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("ESP32-CAM photo server starting.");
    pinMode(FLASH_LED_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_LED_GPIO_NUM, LOW);

    if (!start_camera()) {
        return;
    }

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);   // answer photo requests quickly
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("Connecting to %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.printf("Connected. Photo URL: http://%s/capture\n", WiFi.localIP().toString().c_str());

    start_server();
}

void loop() {
    static uint32_t last_report = 0;
    if (millis() - last_report > 10000) {
        last_report = millis();
        Serial.printf("Photo URL: http://%s/capture | Wi-Fi %d dBm | %lu photos served\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI(),
                      (unsigned long) photos_served);
    }
    delay(100);
}