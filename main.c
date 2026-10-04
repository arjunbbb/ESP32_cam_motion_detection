
#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_http_server.h"

// ---------------- User settings ----------------
const char *WIFI_SSID     = "A";
const char *WIFI_PASSWORD = "1234567890";

#define LED_PIN    33     // on-board red LED (active LOW)
#define ALERT_PIN  13     // optional output for a buzzer/relay driver; set -1 to disable

// ---------------- Detection tuning ----------------
constexpr int   W = 320, H = 240;          // FRAMESIZE_QVGA
constexpr int   BS = 8;                    // block size in pixels
constexpr int   GW = W / BS, GH = H / BS;  // 40 x 30 block grid

constexpr float BG_ALPHA          = 0.15f; // reference update rate (1.0 = previous frame)
constexpr int   BLOCK_THRESH      = 14;    // brightness change (0-255) for a block to count
constexpr int   MIN_NEIGHBORS     = 1;     // active neighbours needed to keep a block (noise filter)
constexpr int   MIN_BLOCKS        = 3;     // min blocks to call it motion
constexpr float LIGHT_CHANGE_FRAC = 0.60f; // > 60% blocks changed = lighting change, resync
constexpr int   CONFIRM_FRAMES    = 2;     // consecutive detections before "motion"
constexpr uint32_t HOLD_MS        = 1500;  // keep motion state after last detection
constexpr float BOX_SMOOTH        = 0.5f;  // 1.0 = no smoothing
constexpr int   WARMUP_FRAMES     = 15;    // let auto exposure settle
constexpr int   JPEG_QUALITY      = 12;    // 0-63, lower = better quality

// ---------------- Shared state ----------------
struct MotionEvent { bool active; int x0, y0, x1, y1; uint32_t ms; };
struct MotionStatus { bool motion; int x0, y0, x1, y1; };

static QueueHandle_t     eventQ;
static SemaphoreHandle_t jpgMutex, statusMutex;

static uint8_t *latestJpg = nullptr;
static size_t   latestLen = 0;
static volatile uint32_t jpgSeq = 0;
static volatile int      streamClients = 0;

static MotionStatus gStatus = {false, 0, 0, 0, 0};
static volatile uint32_t gFrames = 0, gEvents = 0;

static httpd_handle_t ctrlServer = nullptr, streamServer = nullptr;

// ---------------- Camera ----------------
static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = 5;  c.pin_d1 = 18; c.pin_d2 = 19; c.pin_d3 = 21;
  c.pin_d4 = 36; c.pin_d5 = 39; c.pin_d6 = 34; c.pin_d7 = 35;
  c.pin_xclk = 0;  c.pin_pclk = 22; c.pin_vsync = 25; c.pin_href = 23;
  c.pin_sscb_sda = 26; c.pin_sscb_scl = 27;
  c.pin_pwdn = 32; c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;   // 1 byte/pixel, no JPEG decode needed
  c.frame_size   = FRAMESIZE_QVGA;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

// ---------------- Motion detection ----------------
static float   bg[GW * GH];
static uint8_t cur[GW * GH];
static uint8_t act[GW * GH];

static void blockMeans(const uint8_t *img, uint8_t *out) {
  for (int by = 0; by < GH; by++) {
    for (int bx = 0; bx < GW; bx++) {
      const uint8_t *p = img + (by * BS) * W + bx * BS;
      uint32_t s = 0;
      for (int y = 0; y < BS; y++, p += W)
        for (int x = 0; x < BS; x++) s += p[x];
      out[by * GW + bx] = s / (BS * BS);
    }
  }
}

// Returns true if motion found; fills the pixel-space box.
static bool findMotion(int &x0, int &y0, int &x1, int &y1) {
  int activeTotal = 0;
  for (int i = 0; i < GW * GH; i++) {
    float diff = fabsf((float)cur[i] - bg[i]);
    act[i] = diff > BLOCK_THRESH;
    activeTotal += act[i];
    bg[i] += BG_ALPHA * ((float)cur[i] - bg[i]);   // update reference
  }

  // Whole scene changed (auto exposure / lights): resync, don't report motion
  if (activeTotal > (int)(GW * GH * LIGHT_CHANGE_FRAC)) {
    for (int i = 0; i < GW * GH; i++) bg[i] = cur[i];
    return false;
  }

  int minx = GW, miny = GH, maxx = -1, maxy = -1, count = 0;
  for (int by = 0; by < GH; by++) {
    for (int bx = 0; bx < GW; bx++) {
      if (!act[by * GW + bx]) continue;
      int nb = 0;
      for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
          if (!dx && !dy) continue;
          int nx = bx + dx, ny = by + dy;
          if (nx < 0 || ny < 0 || nx >= GW || ny >= GH) continue;
          nb += act[ny * GW + nx];
        }
      if (nb < MIN_NEIGHBORS) continue;          // isolated noise block
      count++;
      if (bx < minx) minx = bx;  if (bx > maxx) maxx = bx;
      if (by < miny) miny = by;  if (by > maxy) maxy = by;
    }
  }
  if (count < MIN_BLOCKS) return false;

  x0 = minx * BS;        y0 = miny * BS;
  x1 = (maxx + 1) * BS - 1;  y1 = (maxy + 1) * BS - 1;
  return true;
}

static inline void putPx(uint8_t *img, int x, int y, uint8_t v) {
  if (x >= 0 && y >= 0 && x < W && y < H) img[y * W + x] = v;
}

// 2-pixel thick white square outline
static void drawBox(uint8_t *img, int x0, int y0, int x1, int y1) {
  for (int t = 0; t < 2; t++) {
    for (int x = x0; x <= x1; x++) { putPx(img, x, y0 + t, 255); putPx(img, x, y1 - t, 255); }
    for (int y = y0; y <= y1; y++) { putPx(img, x0 + t, y, 255); putPx(img, x1 - t, y, 255); }
  }
}

// ---------------- JPEG sharing between tasks ----------------
static void publishJpeg(uint8_t *jpg, size_t len) {
  if (xSemaphoreTake(jpgMutex, portMAX_DELAY) == pdTRUE) {
    free(latestJpg);
    latestJpg = jpg;
    latestLen = len;
    jpgSeq++;
    xSemaphoreGive(jpgMutex);
  } else {
    free(jpg);
  }
}

// Copies the newest JPEG (if newer than *seq) into a caller-owned buffer.
static size_t copyLatestJpeg(uint8_t **dst, size_t *cap, uint32_t *seq) {
  size_t n = 0;
  if (xSemaphoreTake(jpgMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (latestJpg && jpgSeq != *seq) {
      if (latestLen > *cap) {
        uint8_t *nb = (uint8_t *)realloc(*dst, latestLen + 1024);
        if (nb) { *dst = nb; *cap = latestLen + 1024; }
      }
      if (latestLen <= *cap) {
        memcpy(*dst, latestJpg, latestLen);
        n = latestLen;
        *seq = jpgSeq;
      }
    }
    xSemaphoreGive(jpgMutex);
  }
  return n;
}

// ---------------- Tasks ----------------
static void captureTask(void *) {
  // Warm-up so auto exposure/gain settle before we build the reference
  for (int i = 0; i < WARMUP_FRAMES; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    vTaskDelay(pdMS_TO_TICKS(30));
  }

  bool bgReady = false, active = false, haveBox = false;
  int hits = 0;
  uint32_t lastSeen = 0;
  float sx0 = 0, sy0 = 0, sx1 = 0, sy1 = 0;   // smoothed box

  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (fb->width != W || fb->height != H) { esp_camera_fb_return(fb); continue; }

    blockMeans(fb->buf, cur);
    uint32_t now = millis();

    if (!bgReady) {
      for (int i = 0; i < GW * GH; i++) bg[i] = cur[i];
      bgReady = true;
    } else {
      int bx0, by0, bx1, by1;
      if (findMotion(bx0, by0, bx1, by1)) {
        hits++;
        lastSeen = now;
        if (!haveBox) { sx0 = bx0; sy0 = by0; sx1 = bx1; sy1 = by1; haveBox = true; }
        else {
          sx0 += BOX_SMOOTH * (bx0 - sx0);  sy0 += BOX_SMOOTH * (by0 - sy0);
          sx1 += BOX_SMOOTH * (bx1 - sx1);  sy1 += BOX_SMOOTH * (by1 - sy1);
        }
      } else {
        hits = 0;
      }

      if (!active && hits >= CONFIRM_FRAMES) {
        active = true;
        gEvents++;
        MotionEvent ev = {true, (int)sx0, (int)sy0, (int)sx1, (int)sy1, now};
        xQueueSend(eventQ, &ev, 0);
      } else if (active && (now - lastSeen) > HOLD_MS) {
        active = false;
        haveBox = false;
        MotionEvent ev = {false, 0, 0, 0, 0, now};
        xQueueSend(eventQ, &ev, 0);
      }
    }

    if (active && haveBox) drawBox(fb->buf, (int)sx0, (int)sy0, (int)sx1, (int)sy1);

    if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      gStatus = {active, (int)sx0, (int)sy0, (int)sx1, (int)sy1};
      xSemaphoreGive(statusMutex);
    }

    // Only spend CPU on JPEG encoding while someone is watching
    if (streamClients > 0) {
      uint8_t *jpg = nullptr;
      size_t jlen = 0;
      if (fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_GRAYSCALE,
                  JPEG_QUALITY, &jpg, &jlen) && jpg) {
        publishJpeg(jpg, jlen);
      }
    }

    esp_camera_fb_return(fb);
    gFrames++;
    vTaskDelay(1);   // yield to lower-priority tasks
  }
}

static void alertTask(void *) {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);            // off (active LOW)
  if (ALERT_PIN >= 0) { pinMode(ALERT_PIN, OUTPUT); digitalWrite(ALERT_PIN, LOW); }

  MotionEvent ev;
  for (;;) {
    if (xQueueReceive(eventQ, &ev, portMAX_DELAY) == pdTRUE) {
      digitalWrite(LED_PIN, ev.active ? LOW : HIGH);
      if (ALERT_PIN >= 0) digitalWrite(ALERT_PIN, ev.active ? HIGH : LOW);
      if (ev.active)
        Serial.printf("[%lu ms] MOTION  box=(%d,%d)-(%d,%d)\n", ev.ms, ev.x0, ev.y0, ev.x1, ev.y1);
      else
        Serial.printf("[%lu ms] motion ended\n", ev.ms);
      // Hook point: send Telegram/MQTT message, save to SD, etc.
    }
  }
}

static void monitorTask(void *) {
  uint32_t lastFrames = 0;
  int stalled = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    uint32_t f = gFrames;
    Serial.printf("fps=%.1f  heap=%u  psram=%u  rssi=%d  clients=%d\n",
                  (f - lastFrames) / 5.0f, ESP.getFreeHeap(), ESP.getFreePsram(),
                  WiFi.RSSI(), (int)streamClients);

    // Watchdog: restart if the capture task stops producing frames
    stalled = (f == lastFrames) ? stalled + 1 : 0;
    if (stalled >= 3) { Serial.println("Capture stalled, restarting"); ESP.restart(); }
    lastFrames = f;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi lost, reconnecting...");
      WiFi.reconnect();
    }
  }
}

// ---------------- Web server ----------------
static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-CAM Motion</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:12px}
 img{width:100%;max-width:640px;image-rendering:pixelated;border:2px solid #333;border-radius:6px}
 #s{font-size:1.2rem;margin:10px;padding:8px;border-radius:6px;display:inline-block;min-width:200px}
 .idle{background:#234}.hit{background:#a22}
</style></head><body>
<h2>ESP32-CAM motion detection</h2>
<div id="s" class="idle">Idle</div><br>
<img id="v" alt="stream">
<p id="i" style="color:#888"></p>
<script>
 document.getElementById('v').src='http://'+location.hostname+':81/stream';
 async function poll(){
  try{
   const r=await fetch('/status'); const j=await r.json();
   const s=document.getElementById('s');
   s.textContent=j.motion?'MOTION DETECTED':'Idle';
   s.className=j.motion?'hit':'idle';
   document.getElementById('i').textContent='events: '+j.events+' | frames: '+j.frames;
  }catch(e){}
  setTimeout(poll,500);
 }
 poll();
</script></body></html>
)rawliteral";

static esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t statusHandler(httpd_req_t *req) {
  MotionStatus s = {};
  if (xSemaphoreTake(statusMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    s = gStatus;
    xSemaphoreGive(statusMutex);
  }
  char buf[192];
  snprintf(buf, sizeof(buf),
           "{\"motion\":%s,\"box\":[%d,%d,%d,%d],\"events\":%lu,\"frames\":%lu,\"uptime_ms\":%lu}",
           s.motion ? "true" : "false", s.x0, s.y0, s.x1, s.y1,
           (unsigned long)gEvents, (unsigned long)gFrames, (unsigned long)millis());
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t snapshotHandler(httpd_req_t *req) {
  uint8_t *buf = nullptr;
  size_t cap = 0, len = 0;
  uint32_t seq = jpgSeq;          // wait for a fresh frame

  streamClients++;
  for (int i = 0; i < 150 && len == 0; i++) {
    len = copyLatestJpeg(&buf, &cap, &seq);
    if (!len) vTaskDelay(pdMS_TO_TICKS(10));
  }
  streamClients--;

  if (!len) { free(buf); httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");
  esp_err_t r = httpd_resp_send(req, (const char *)buf, len);
  free(buf);
  return r;
}

static const char *STREAM_CT   = "multipart/x-mixed-replace;boundary=frame";
static const char *STREAM_BND  = "\r\n--frame\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, STREAM_CT);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  uint8_t *buf = nullptr;
  size_t cap = 0;
  uint32_t seq = 0;
  esp_err_t res = ESP_OK;

  streamClients++;
  while (res == ESP_OK) {
    size_t len = copyLatestJpeg(&buf, &cap, &seq);
    if (!len) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    char hdr[64];
    int hl = snprintf(hdr, sizeof(hdr), STREAM_PART, (unsigned)len);
    res = httpd_resp_send_chunk(req, STREAM_BND, strlen(STREAM_BND));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, hdr, hl);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)buf, len);
  }
  streamClients--;
  free(buf);
  return res;
}

static void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.core_id = 0;                 // keep HTTP work off the capture core
  cfg.stack_size = 8192;
  cfg.max_uri_handlers = 8;

  httpd_uri_t uIndex  = {"/",         HTTP_GET, indexHandler,    nullptr};
  httpd_uri_t uStatus = {"/status",   HTTP_GET, statusHandler,   nullptr};
  httpd_uri_t uSnap   = {"/snapshot", HTTP_GET, snapshotHandler, nullptr};
  httpd_uri_t uStream = {"/stream",   HTTP_GET, streamHandler,   nullptr};

  if (httpd_start(&ctrlServer, &cfg) == ESP_OK) {
    httpd_register_uri_handler(ctrlServer, &uIndex);
    httpd_register_uri_handler(ctrlServer, &uStatus);
    httpd_register_uri_handler(ctrlServer, &uSnap);
  }

  // The stream handler blocks its server's worker, so it gets its own instance on :81
  cfg.server_port += 1;
  cfg.ctrl_port   += 1;
  if (httpd_start(&streamServer, &cfg) == ESP_OK) {
    httpd_register_uri_handler(streamServer, &uStream);
  }
}

// ---------------- Arduino entry points ----------------
void setup() {
  Serial.begin(115200);
  Serial.println("\nESP32-CAM motion detection (FreeRTOS)");

  if (!psramFound()) {
    Serial.println("PSRAM not found - enable PSRAM in Tools menu / check the board.");
    delay(3000);
    ESP.restart();
  }
  if (!initCamera()) { delay(3000); ESP.restart(); }

  jpgMutex    = xSemaphoreCreateMutex();
  statusMutex = xSemaphoreCreateMutex();
  eventQ      = xQueueCreate(8, sizeof(MotionEvent));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);            // lower latency for streaming
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > 20000) { Serial.println("\nWi-Fi failed, restarting"); ESP.restart(); }
    delay(400);
    Serial.print(".");
  }
  Serial.printf("\nConnected. Open http://%s/\n", WiFi.localIP().toString().c_str());

  startServers();

  // Camera driver was initialised on this core (1), so capture stays on core 1
  xTaskCreatePinnedToCore(captureTask, "capture", 8192, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(alertTask,   "alert",   3072, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(monitorTask, "monitor", 4096, nullptr, 1, nullptr, 0);
}

void loop() {
  vTaskDelete(NULL);   // everything runs in FreeRTOS tasks; the Arduino loop task isn't needed
}
