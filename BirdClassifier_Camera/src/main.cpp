#include <Arduino.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include <HTTPClient.h>

// AI Thinker pin definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid     = "";
const char* password = "";

WiFiServer server(80);

struct Config {
  int motionThreshold;
  float motionSensitivity;
  int motionCheckMs;
  bool motionEnabled;
  int jpegQuality;
  int framesize;
  char serverIp[16];
  int serverPort;
};

Config cfg = {
  .motionThreshold   = 20,
  .motionSensitivity = 0.05f,
  .motionCheckMs     = 500,
  .motionEnabled     = true,
  .jpegQuality       = 10,
  .framesize         = 8,    // FRAMESIZE_VGA
  .serverIp          = "192.168.1.50",
  .serverPort        = 5000,
};

bool sdReady = false;

// SD card config

void loadConfig() {
  if (!sdReady) return;

  File f = SD_MMC.open("/config.json", FILE_READ);
  if (!f) {
    Serial.println("config.json not found, using current values");
    return;
  }

  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    Serial.printf("Config parse error: %s — keeping current values\n", err.c_str());
    return;
  }

  if (doc.containsKey("motion_sensitivity")) cfg.motionSensitivity = doc["motion_sensitivity"];
  if (doc.containsKey("server_port"))        cfg.serverPort        = doc["server_port"];

  Serial.println("Config loaded from SD:");
  Serial.printf("motion_enabled:     %s\n",  cfg.motionEnabled ? "true" : "false");
  Serial.printf("motion_threshold:   %d\n",  cfg.motionThreshold);
  Serial.printf("motion_sensitivity: %.2f\n",cfg.motionSensitivity);
  Serial.printf("motion_check_ms:    %d\n",  cfg.motionCheckMs);
  Serial.printf("jpeg_quality:       %d\n",  cfg.jpegQuality);
  Serial.printf("framesize:          %d\n",  cfg.framesize);

  Serial.printf("server_ip : %s\n", cfg.serverIp);
  // NOTE: fixed from original — was %s with an int argument
  Serial.printf("server_port : %d\n", cfg.serverPort);
}

void saveConfig() {
  if (!sdReady) return;

  StaticJsonDocument<512> doc;
  doc["motion_threshold"]   = cfg.motionThreshold;
  doc["motion_sensitivity"]= cfg.motionSensitivity;
  doc["motion_check_ms"] = cfg.motionCheckMs;
  doc["motion_enabled"]= cfg.motionEnabled;
  doc["jpeg_quality"] = cfg.jpegQuality;
  doc["framesize"] = cfg.framesize;
  doc["server_ip"] = cfg.serverIp;
  doc["server_port"]= cfg.serverPort;

  File f = SD_MMC.open("/config.json", FILE_WRITE);
  if (!f) {
    Serial.println("Failed to open config.json for writing");
    return;
  }
  serializeJsonPretty(doc, f);
  f.close();
  Serial.println("Config saved to SD");
}

void postFrame(camera_fb_t* fb) {
  HTTPClient http;
  char url[64];
  snprintf(url, sizeof(url), "http://%s:%d/upload", cfg.serverIp, cfg.serverPort);

  http.begin(url);
  http.addHeader("Content-Type", "image/jpeg");

  int httpCode = http.POST(fb->buf, fb->len);
  if (httpCode > 0) {
    Serial.printf("POST %s → %d\n", url, httpCode);
  } else {
    Serial.printf("POST failed: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

// Apply camera settings that need a sensor call

void applyCameraSettings() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  s->set_quality(s, cfg.jpegQuality);
  s->set_framesize(s, (framesize_t)cfg.framesize);
}

// Motion detection state

volatile bool motionFlag = false;

#define MD_WIDTH  80
#define MD_HEIGHT 60
#define MD_PIXELS (MD_WIDTH * MD_HEIGHT)

uint8_t  refFrame[MD_PIXELS];
bool     refFrameReady  = false;

unsigned long lastMotionCheck  = 0;
unsigned long lastConfigCheck  = 0;
const unsigned long CONFIG_POLL_MS = 10000; // re-read SD every 10 seconds

void checkMotion() {
  if (!cfg.motionEnabled) return;

  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;

  s->set_framesize(s, FRAMESIZE_QQVGA);
  s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
  delay(150);

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_framesize(s, (framesize_t)cfg.framesize);
    return;
  }

  if (!refFrameReady) {
    for (int y = 0; y < MD_HEIGHT; y++) {
      for (int x = 0; x < MD_WIDTH; x++) {
        int sx = x * 2, sy = y * 2;
        refFrame[y * MD_WIDTH + x] = (
          fb->buf[sy * 160 + sx] +
          fb->buf[sy * 160 + sx + 1] +
          fb->buf[(sy+1) * 160 + sx] +
          fb->buf[(sy+1) * 160 + sx + 1]
        ) / 4;
      }
    }
    refFrameReady = true;
    esp_camera_fb_return(fb);
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_framesize(s, (framesize_t)cfg.framesize);
    return;
  }

  int changedPixels = 0;
  for (int y = 0; y < MD_HEIGHT; y++) {
    for (int x = 0; x < MD_WIDTH; x++) {
      int sx = x * 2, sy = y * 2;
      uint8_t curr = (
        fb->buf[sy * 160 + sx] +
        fb->buf[sy * 160 + sx + 1] +
        fb->buf[(sy+1) * 160 + sx] +
        fb->buf[(sy+1) * 160 + sx + 1]
      ) / 4;

      if (abs((int)curr - (int)refFrame[y * MD_WIDTH + x]) > cfg.motionThreshold) {
        changedPixels++;
      }

      refFrame[y * MD_WIDTH + x] = (uint8_t)(
        0.8f * refFrame[y * MD_WIDTH + x] + 0.2f * curr
      );
    }
  }

  esp_camera_fb_return(fb);
  s->set_pixformat(s, PIXFORMAT_JPEG);
  s->set_framesize(s, (framesize_t)cfg.framesize);
  delay(50);

  float changedFraction = (float)changedPixels / MD_PIXELS;
  if (changedFraction > cfg.motionSensitivity) {
    motionFlag = true;
    Serial.printf("Motion detected: %.1f%% pixels changed\n", changedFraction * 100);

    // Switch back to JPEG/VGA before capturing the frame to send
    s->set_pixformat(s, PIXFORMAT_JPEG);
    s->set_framesize(s, (framesize_t)cfg.framesize);
    delay(50); // let sensor settle into new mode

    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      postFrame(fb);
      esp_camera_fb_return(fb);
    }
  }
}

// HTTP handler

void handleClient(WiFiClient client) {
  String request = "";
  while (client.connected()) {
    if (client.available()) {
      char c = client.read();
      request += c;
      if (request.endsWith("\r\n\r\n")) break;
    }
  }

  // GET /capture — full JPEG frame
  if (request.indexOf("GET /capture") >= 0) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      client.println("HTTP/1.1 500 Internal Server Error\r\n\r\n");
      client.stop();
      return;
    }
    client.printf(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: image/jpeg\r\n"
      "Content-Length: %u\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "\r\n",
      fb->len
    );
    client.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);

  // GET /motion — poll motion flag
  } else if (request.indexOf("GET /motion") >= 0) {
    String json = motionFlag ? "{\"motion\":true}" : "{\"motion\":false}";
    motionFlag = false;
    client.printf(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: %u\r\n"
      "Access-Control-Allow-Origin: *\r\n"
      "\r\n"
      "%s",
      json.length(), json.c_str()
    );

  // GET /config — read current config
  } else if (request.indexOf("GET /config") >= 0) {
    StaticJsonDocument<512> doc;
    doc["motion_threshold"]   = cfg.motionThreshold;
    doc["motion_sensitivity"] = cfg.motionSensitivity;
    doc["motion_check_ms"]    = cfg.motionCheckMs;
    doc["motion_enabled"]     = cfg.motionEnabled;
    doc["jpeg_quality"]       = cfg.jpegQuality;
    doc["framesize"]          = cfg.framesize;
    String json;
    serializeJson(doc, json);
    client.printf(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: %u\r\n"
      "\r\n"
      "%s",
      json.length(), json.c_str()
    );

  // POST /config — update config values
  } else if (request.indexOf("POST /config") >= 0) {
    // Read POST body (comes after the headers)
    String body = "";
    while (client.connected() && client.available()) {
      body += (char)client.read();
    }

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, body);
    if (err) {
      String resp = "{\"error\":\"bad json\"}";
      client.printf(
        "HTTP/1.1 400 Bad Request\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n\r\n%s",
        resp.length(), resp.c_str()
      );
    } else {
      if (doc.containsKey("motion_threshold"))   cfg.motionThreshold   = doc["motion_threshold"];
      if (doc.containsKey("motion_sensitivity")) cfg.motionSensitivity = doc["motion_sensitivity"];
      if (doc.containsKey("motion_check_ms"))    cfg.motionCheckMs     = doc["motion_check_ms"];
      if (doc.containsKey("motion_enabled"))     cfg.motionEnabled     = doc["motion_enabled"];
      if (doc.containsKey("jpeg_quality"))       cfg.jpegQuality       = doc["jpeg_quality"];
      if (doc.containsKey("framesize"))          cfg.framesize         = doc["framesize"];

      saveConfig();          // persist to SD immediately
      applyCameraSettings(); // apply sensor changes live

      String resp = "{\"status\":\"ok\"}";
      client.printf(
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %u\r\n\r\n%s",
        resp.length(), resp.c_str()
      );
    }

  // GET /status — debug overview
  } else if (request.indexOf("GET /status") >= 0) {
    String json = "{\"motion_flag\":"    + String(motionFlag ? "true" : "false") +
                  ",\"motion_enabled\":" + String(cfg.motionEnabled ? "true" : "false") +
                  ",\"threshold\":"      + String(cfg.motionThreshold) +
                  ",\"sensitivity\":"    + String(cfg.motionSensitivity) +
                  ",\"ref_ready\":"      + String(refFrameReady ? "true" : "false") +
                  ",\"sd_ready\":"       + String(sdReady ? "true" : "false") + "}";
    client.printf(
      "HTTP/1.1 200 OK\r\n"
      "Content-Type: application/json\r\n"
      "Content-Length: %u\r\n\r\n%s",
      json.length(), json.c_str()
    );

  } else {
    client.println("HTTP/1.1 404 Not Found\r\n\r\n");
  }

  client.stop();
}

// Setup

void setup() {
  Serial.begin(115200);

  // SD card — mount before WiFi and camera so config is ready
  if (!SD_MMC.begin("/sdcard", true)) { // true = 1-bit mode for AI Thinker
    Serial.println("SD mount failed — using default config");
  } else {
    sdReady = true;
    Serial.println("SD mounted");
  }

  loadConfig(); // load from SD before anything else uses cfg

  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println("\nConnected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  camera_config_t camera_config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sccb_sda   = SIOD_GPIO_NUM,
    .pin_sccb_scl   = SIOC_GPIO_NUM,
    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,
    .xclk_freq_hz   = 20000000,
    .ledc_timer     = LEDC_TIMER_0,
    .ledc_channel   = LEDC_CHANNEL_0,
    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size     = FRAMESIZE_VGA,
    .jpeg_quality   = 10,
    .fb_count       = 1,
    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
  };

  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    // ====== RECONSTRUCTED FROM HERE DOWN — uploaded file was truncated ======
    while (true) { delay(1000); }
  }

  applyCameraSettings();

  server.begin();
  Serial.println("HTTP server started");
}

// Loop (reconstructed — not in the uploaded file)

void loop() {
  WiFiClient client = server.available();
  if (client) handleClient(client);

  unsigned long now = millis();

  if (now - lastMotionCheck >= (unsigned long)cfg.motionCheckMs) {
    lastMotionCheck = now;
    checkMotion();
  }

  if (now - lastConfigCheck >= CONFIG_POLL_MS) {
    lastConfigCheck = now;
    loadConfig();
  }
}