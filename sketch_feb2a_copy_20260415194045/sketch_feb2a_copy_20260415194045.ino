#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <SPIFFS.h>
#include <FS.h>

//WiFi configuration
const char* ssid = "";
const char* password = "";

//Laptop receiver
const char* laptopIP = "192.168.1.129";
const int   laptopPort = 5000;

// Motion detection capture mode configurations
#define CAPTURE_PIXEL_FORMAT  PIXFORMAT_GRAYSCALE
#define CAPTURE_FRAME_SIZE    FRAMESIZE_QQVGA       // 160×120 = 19,200 bytes

#define FRAME_W 160
#define FRAME_H 120
#define FRAME_PIXELS (FRAME_W * FRAME_H)

//Downsample settings
#define DOWNSAMPLE_FACTOR 2
#define DS_W (FRAME_W / DOWNSAMPLE_FACTOR) //80
#define DS_H (FRAME_H / DOWNSAMPLE_FACTOR) //60
#define DS_PIXELS (DS_W * DS_H)

//Motion settings
const float MOTION_THRESHOLD = 10.0f;  //lower = more sensitive
const unsigned long MOTION_COOLDOWN_MS = 5000; // 5 fps
const unsigned long MOTION_POLL_DELAY_MS = 200;   // ~5 fps

// Burst settings
const int BURST_COUNT = 3;
const int BURST_DELAY_MS = 1500;   // 1.5 seconds

//AsyncTCP settings
#define CONFIG_ASYNC_TCP_MAX_ACK_TIME 3000
#define CONFIG_ASYNC_TCP_PRIORITY 10
#define CONFIG_ASYNC_TCP_RUNNING_CORE 1

// spiffs last photo path
#define FILE_PHOTO "/photo.jpg"

// Camera pins (AI thinker ESP32-CAM)
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

AsyncWebServer server(80);

//ROI
struct ROI {
  int x;
  int y;
  int w;
  int h;
};

enum CaptureSource {
  SOURCE_MANUAL,
  SOURCE_MOTION,
  SOURCE_BURST
};

//motion ROI in downsampled coordinates (80x60, factor = 2)
ROI motionROI = {20, 15, 30, 20};

static uint8_t dsPrev[DS_PIXELS];
static uint8_t dsCurr[DS_PIXELS];
static bool hasPrevFrame = false;

volatile boolean takeNewPhoto = false;
volatile boolean motionDetectionActive = false;
volatile boolean captureBurst = false;

CaptureSource pendingCaptureSource = SOURCE_MANUAL;
unsigned long lastMotionTriggerMs = 0;

//HTML

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { text-align:center; font-family: sans-serif; }
    #motionBtn { background-color: #888; color: white; padding: 6px 12px; border: none; cursor: pointer; }
    #motionBtn.active { background-color: #c0392b; }
    #photo { max-width: 90%; max-height: 80vh; width: auto; height: auto; border: 1px solid #ccc; }
    button:disabled { opacity: 0.4; cursor: not-allowed; }
    .roiBox { margin-top: 12px; }
    input { width: 60px; }
  </style>
</head>
<body>
  <div id="container">
    <h2>ESP32-CAM</h2>
    <p id="statusText" style="font-weight:bold; font-size:14px;">IDLE</p>
    <p>
      <button id="captureBtn" onclick="capturePhoto()">CAPTURE PHOTO</button>
      <button onclick="location.reload()">REFRESH PAGE</button>
      <button id="motionBtn" onclick="toggleMotion()">MOTION DETECTION: OFF</button>
      <button onclick="sendRequest('/resetcam', 2, function(){})">RESET CAMERA</button>
    </p>

    <div id="sensorReadings" style="margin-top:10px; font-size:14px;">
      <span>Chip Temp: <b id="chipTemp">--</b> C</span>
      &nbsp;&nbsp;
      <span>Heap: <b id="heapUsed">--</b>% used (<b id="freeHeap">--</b>KB free of <b id="totalHeap">--</b>KB)</span>
      &nbsp;&nbsp;
      <span>PSRAM: <b id="freePsram">--</b>KB free of <b id="totalPsram">--</b>KB</span>
    </div>

    <div class="roiBox">
      <h3>Motion ROI</h3>
      X: <input id="roiX" type="number">
      Y: <input id="roiY" type="number">
      W: <input id="roiW" type="number">
      H: <input id="roiH" type="number">
      <button onclick="saveROI()">SAVE ROI</button>
    </div>
  </div>

  <div style="margin-top:15px;">
    <img src="/saved-photo" id="photo">
  </div>

<script>
  var motionActive = false;

  function setStatus(state, text) {
    var el = document.getElementById("statusText");
    el.textContent = text;
    el.style.color = state === "ready" ? "green" : state === "busy" ? "orange" : "red";
  }

  function setBusy(busy) {
    document.getElementById("captureBtn").disabled = busy;
    setStatus(busy ? "busy" : "ready", busy ? "BUSY..." : "IDLE");
  }

  function sendRequest(url, retries, onSuccess) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.timeout = 5000;

    xhr.onload = function() {
      if (xhr.status === 200) {
        onSuccess(xhr.responseText);
      } else if (retries > 0) {
        setTimeout(function() { sendRequest(url, retries - 1, onSuccess); }, 1000);
      } else {
        setStatus("error", "FAILED");
        setBusy(false);
      }
    };

    xhr.onerror = function() {
      if (retries > 0) {
        setTimeout(function() { sendRequest(url, retries - 1, onSuccess); }, 1000);
      } else {
        setStatus("error", "CONNECTION ERROR");
        setBusy(false);
      }
    };

    xhr.ontimeout = function() {
      if (retries > 0) {
        setTimeout(function() { sendRequest(url, retries - 1, onSuccess); }, 1000);
      } else {
        setStatus("error", "TIMED OUT");
        setBusy(false);
      }
    };

    xhr.send();
  }

  function capturePhoto() {
    document.getElementById("captureBtn").dataset.waiting = "true";
    setBusy(true);
    sendRequest("/capture", 3, function() {});
  }

  function toggleMotion() {
    var btn = document.getElementById("motionBtn");
    btn.disabled = true;

    sendRequest("/motion", 3, function(resp) {
      motionActive = resp.indexOf("ON") !== -1;
      btn.textContent = motionActive ? "MOTION DETECTION: ON" : "MOTION DETECTION: OFF";
      btn.classList.toggle("active", motionActive);
      btn.disabled = false;
    });
  }

  function pollStatus() {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/status", true);
    xhr.timeout = 2000;

    xhr.onload = function() {
      if (xhr.status === 200) {
        var busy = xhr.responseText.trim() === "busy";
        setBusy(busy);

        if (!busy && document.getElementById("captureBtn").dataset.waiting === "true") {
          document.getElementById("photo").src = "/saved-photo?t=" + new Date().getTime();
          document.getElementById("captureBtn").dataset.waiting = "false";
        }
      }
    };

    xhr.onerror = function() {
      setStatus("error", "NO CONNECTION");
    };

    xhr.send();
  }

  function pollSensors() {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/sensors", true);
    xhr.timeout = 2000;

    xhr.onload = function() {
      if (xhr.status === 200) {
        var data = JSON.parse(xhr.responseText);
        document.getElementById("chipTemp").textContent = data.temp;
        document.getElementById("freeHeap").textContent = data.freeHeap;
        document.getElementById("totalHeap").textContent = data.totalHeap;
        document.getElementById("heapUsed").textContent = data.heapUsed;
        document.getElementById("freePsram").textContent = data.freePsram;
        document.getElementById("totalPsram").textContent = data.totalPsram;
      }
    };

    xhr.send();
  }

  function loadROI() {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/getroi", true);
    xhr.onload = function() {
      if (xhr.status === 200) {
        var data = JSON.parse(xhr.responseText);
        document.getElementById("roiX").value = data.x;
        document.getElementById("roiY").value = data.y;
        document.getElementById("roiW").value = data.w;
        document.getElementById("roiH").value = data.h;
      }
    };
    xhr.send();
  }

  function saveROI() {
    var x = document.getElementById("roiX").value;
    var y = document.getElementById("roiY").value;
    var w = document.getElementById("roiW").value;
    var h = document.getElementById("roiH").value;

    sendRequest("/setroi?x=" + x + "&y=" + y + "&w=" + w + "&h=" + h, 2, function() {
      setStatus("ready", "ROI SAVED");
      setTimeout(function() { setStatus("ready", "IDLE"); }, 1000);
    });
  }

  setInterval(pollStatus, 1000);
  setInterval(pollSensors, 2000);
  loadROI();
</script>
</body>
</html>
)rawliteral";

// Camera Config

#if ESP_CAMERA_SUPPORTED
static camera_config_t camera_config = {
  .pin_pwdn= PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sccb_sda = SIOD_GPIO_NUM,
  .pin_sccb_scl = SIOC_GPIO_NUM,
  
  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,

  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format =CAPTURE_PIXEL_FORMAT,  
  .frame_size = CAPTURE_FRAME_SIZE,
  .jpeg_quality = 12, 

  .fb_count =1,
  .fb_location =  CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};
#endif

//Utility
float getChipTemp() {
  return temperatureRead();
}
void clampROI(ROI &roi) {
  if (roi.x < 0) roi.x = 0;
  if (roi.y < 0) roi.y = 0;
  if (roi.x >= DS_W) roi.x = DS_W - 1;
  if (roi.y >= DS_H) roi.y = DS_H - 1;

  if (roi.w < 1) roi.w = 1;
  if (roi.h < 1) roi.h = 1;

  if (roi.x + roi.w > DS_W) roi.w = DS_W - roi.x;
  if (roi.y + roi.h > DS_H) roi.h = DS_H - roi.y;
}
const char* captureSourceToString(CaptureSource source) {
  switch (source) {
    case SOURCE_MOTION: return "motion";
    case SOURCE_BURST:  return "burst";
    case SOURCE_MANUAL:
    default:            return "manual";
  }
}

//Frame processing
void downsample(const uint8_t* src, uint8_t* dst) {
  for (int y = 0; y < DS_H; y++) {
    for (int x = 0; x < DS_W; x++) {
      const int s = DOWNSAMPLE_FACTOR;
      uint32_t sum = 0;

      for (int dy = 0; dy < s; dy++) {
        for (int dx = 0; dx < s; dx++) {
          sum += src[(y * s + dy) * FRAME_W + (x * s + dx)];
        }
      }

      dst[y * DS_W + x] = sum / (s * s);
    }
  }
}

float compareFramesROI(const ROI& roi) {
  uint32_t diff = 0;
  uint32_t count = 0;

  int x0 = max(0, roi.x);
  int y0 = max(0, roi.y);
  int x1 = min(DS_W, roi.x + roi.w);
  int y1 = min(DS_H, roi.y + roi.h);

  for (int y = y0; y < y1; y++) {
    for (int x = x0; x < x1; x++) {
      int i = y * DS_W + x;
      diff += abs((int)dsCurr[i] - (int)dsPrev[i]);
      count++;
    }
  }

  if (count == 0) return 0.0f;
  return (float)diff / count;
}

bool detectMotionFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to grab frame for motion detection");
    return false;
  }

  downsample(fb->buf, dsCurr);
  esp_camera_fb_return(fb);

  if (!hasPrevFrame) {
    memcpy(dsPrev, dsCurr, DS_PIXELS);
    hasPrevFrame = true;
    return false;
  }

  float delta = compareFramesROI(motionROI);
  memcpy(dsPrev, dsCurr, DS_PIXELS);

  Serial.printf("Frame delta: %.2f\n", delta);
  return delta > MOTION_THRESHOLD;
}

//storage
bool fileLooksValid(const char* path) {
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) return false;
  size_t sz = f.size();
  f.close();
  return sz > 100;
}

void copyFile(const char* srcPath, const char* dstPath) {
  File src = SPIFFS.open(srcPath, FILE_READ);
  if (!src) return;

  File dst = SPIFFS.open(dstPath, FILE_WRITE);
  if (!dst) {
    src.close();
    return;
  }

  uint8_t buf[512];
  while (src.available()) {
    int n = src.read(buf, sizeof(buf));
    if (n > 0) dst.write(buf, n);
  }

  src.close();
  dst.close();
}

void clearOldPhotos() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();

  while (file) {
    String name = file.name();
    if (name.startsWith("/photo_")) {
      Serial.printf("Deleting old photo: %s\n", name.c_str());
      SPIFFS.remove(name);
    }
    file = root.openNextFile();
  }

  root.close();
}

//networking
void sendPhoto(const char* filename) {
  File file = SPIFFS.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for sending");
    return;
  }

  WiFiClient client;
  if (!client.connect(laptopIP, laptopPort)) {
    Serial.println("Connection to laptop failed");
    file.close();
    return;
  }

  size_t fileSize = file.size();

  client.printf("POST /receive HTTP/1.1\r\n");
  client.printf("Host: %s:%d\r\n", laptopIP, laptopPort);
  client.printf("Content-Type: image/jpeg\r\n");
  client.printf("Content-Length: %u\r\n", (unsigned int)fileSize);
  client.printf("\r\n");

  uint8_t buf[512];
  while (file.available()) {
    int bytesRead = file.read(buf, sizeof(buf));
    if (bytesRead > 0) client.write(buf, bytesRead);
  }

  file.close();

  unsigned long timeoutStart = millis();
  while (client.connected() && (millis() - timeoutStart < 3000)) {
    if (client.available()) break;
    delay(10);
  }

  client.stop();
  Serial.println("Photo sent to laptop");
}

//Capture
bool capturePhotoToSpiffs(CaptureSource source) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return false;
  }

  char filename[48];
  snprintf(filename, sizeof(filename), "/photo_%s_%lu.jpg", captureSourceToString(source), millis());

  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    esp_camera_fb_return(fb);
    return false;
  }

  uint8_t* jpg_buf = nullptr;
  size_t jpg_len = 0;

  bool converted = fmt2jpg(
    fb->buf,
    fb->len,
    fb->width,
    fb->height,
    PIXFORMAT_GRAYSCALE,
    80,
    &jpg_buf,
    &jpg_len
  );

  esp_camera_fb_return(fb);

  if (!converted || jpg_buf == nullptr) {
    Serial.println("JPEG conversion failed");
    file.close();
    return false;
  }

  file.write(jpg_buf, jpg_len);
  file.close();
  free(jpg_buf);

  if (!fileLooksValid(filename)) {
    Serial.println("Saved file failed validation");
    return false;
  }

  copyFile(filename, FILE_PHOTO);
  sendPhoto(FILE_PHOTO);

  Serial.printf("Saved photo: %s\n", filename);
  return true;
}

void doSingleCapture() {
  CaptureSource source = pendingCaptureSource;

  if (capturePhotoToSpiffs(source)) {
    Serial.println("Single capture complete");
  } else {
    Serial.println("Single capture failed");
  }

  pendingCaptureSource = SOURCE_MANUAL;
  takeNewPhoto = false;
}

void doBurstCapture() {
  for (int i = 0; i < BURST_COUNT; i++) {
    bool ok = capturePhotoToSpiffs(SOURCE_BURST);
    Serial.printf("Burst photo %d/%d: %s\n", i + 1, BURST_COUNT, ok ? "ok" : "failed");
    if (i < BURST_COUNT - 1) delay(BURST_DELAY_MS);
  }

  captureBurst = false;
}

// ============================================================
// Camera / Web setup
// ============================================================

void configureCameraSensor() {
  sensor_t* s = esp_camera_sensor_get();
  if (s == nullptr) return;

  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, 0);
  s->set_aec_value(s, 300);
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)2);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  s->set_lenc(s, 1);
}

bool initCamera() {
#if ESP_CAMERA_SUPPORTED
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  camera_fb_t* pic = esp_camera_fb_get();
  if (pic == nullptr) {
    Serial.println("Camera test frame failed");
    return false;
  }

  Serial.println("=== Frame info ===");
  Serial.printf("Width: %d\n", pic->width);
  Serial.printf("Height: %d\n", pic->height);
  Serial.printf("Length (bytes): %d\n", pic->len);
  Serial.printf("Format: %d\n", pic->format);

  esp_camera_fb_return(pic);

  configureCameraSensor();
  return true;
#else
  return false;
#endif
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", index_html);
  });

  server.on("/capture", HTTP_GET, [](AsyncWebServerRequest* request) {
    pendingCaptureSource = SOURCE_MANUAL;
    takeNewPhoto = true;
    request->send(200, "text/plain", "Taking Photo");
  });

  server.on("/burst", HTTP_GET, [](AsyncWebServerRequest* request) {
    captureBurst = true;
    request->send(200, "text/plain", "Burst started");
  });

  server.on("/motion", HTTP_GET, [](AsyncWebServerRequest* request) {
    motionDetectionActive = !motionDetectionActive;
    if (!motionDetectionActive) {
      hasPrevFrame = false;
    }
    request->send(200, "text/plain", motionDetectionActive ? "Motion ON" : "Motion OFF");
  });

  server.on("/saved-photo", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(SPIFFS, FILE_PHOTO, "image/jpg", false);
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    String state = (takeNewPhoto || captureBurst) ? "busy" : "ready";
    request->send(200, "text/plain", state);
  });

  server.on("/resetcam", HTTP_GET, [](AsyncWebServerRequest* request) {
    configureCameraSensor();
    request->send(200, "text/plain", "Camera reset");
  });

  server.on("/sensors", HTTP_GET, [](AsyncWebServerRequest* request) {
    float chipTemp = getChipTemp();

    uint32_t freeHeap = ESP.getFreeHeap();
    uint32_t totalHeap = ESP.getHeapSize();
    uint32_t freePsram = ESP.getFreePsram();
    uint32_t totalPsram = ESP.getPsramSize();
    uint8_t heapPercent = (uint8_t)(100.0f - ((float)freeHeap / totalHeap * 100.0f));

    String json = "{";
    json += "\"temp\":" + String(chipTemp, 1) + ",";
    json += "\"freeHeap\":" + String(freeHeap / 1024) + ",";
    json += "\"totalHeap\":" + String(totalHeap / 1024) + ",";
    json += "\"heapUsed\":" + String(heapPercent) + ",";
    json += "\"freePsram\":" + String(freePsram / 1024) + ",";
    json += "\"totalPsram\":" + String(totalPsram / 1024);
    json += "}";

    request->send(200, "application/json", json);
  });

  server.on("/getroi", HTTP_GET, [](AsyncWebServerRequest* request) {
    String json = "{";
    json += "\"x\":" + String(motionROI.x) + ",";
    json += "\"y\":" + String(motionROI.y) + ",";
    json += "\"w\":" + String(motionROI.w) + ",";
    json += "\"h\":" + String(motionROI.h);
    json += "}";
    request->send(200, "application/json", json);
  });

  server.on("/setroi", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("x")) motionROI.x = request->getParam("x")->value().toInt();
    if (request->hasParam("y")) motionROI.y = request->getParam("y")->value().toInt();
    if (request->hasParam("w")) motionROI.w = request->getParam("w")->value().toInt();
    if (request->hasParam("h")) motionROI.h = request->getParam("h")->value().toInt();

    clampROI(motionROI);
    request->send(200, "text/plain", "ROI updated");
  });

  server.begin();
}

// ============================================================
// Setup / Loop
// ============================================================

void setup() {
  Serial.begin(115200);

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }

  Serial.println();
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    ESP.restart();
  }

  Serial.println("SPIFFS Info:");
  Serial.printf("Total: %d KB\n", SPIFFS.totalBytes() / 1024);
  Serial.printf("Used:  %d KB\n", SPIFFS.usedBytes() / 1024);
  Serial.printf("Free:  %d KB\n", (SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024);

  clearOldPhotos();

  if (!initCamera()) {
    Serial.println("Camera initialization failed");
    return;
  }

  clampROI(motionROI);
  setupWebServer();
}

void loop() {
  if (motionDetectionActive && !takeNewPhoto && !captureBurst) {
    bool motionDetected = detectMotionFrame();

    if (motionDetected && (millis() - lastMotionTriggerMs >= MOTION_COOLDOWN_MS)) {
      Serial.println("Motion detected");
      pendingCaptureSource = SOURCE_MOTION;
      takeNewPhoto = true;
      lastMotionTriggerMs = millis();
    }
  } else if (!motionDetectionActive) {
    hasPrevFrame = false;
  }

  if (takeNewPhoto) {
    doSingleCapture();
  }

  if (captureBurst) {
    doBurstCapture();
  }

  delay(MOTION_POLL_DELAY_MS);
}
