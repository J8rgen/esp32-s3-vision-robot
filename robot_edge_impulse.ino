// Main loop seletus:
// teeb kaameraga kaadri ja käivitab edge impulsei mudeli
// kui mõõdutops leitakse, hakka selle poole sõitma
// kasuta ultraheliandurit, et robot X cm kaugusel peatada
// siis sõidab veel natuke otse (ajaga), haarab tassi ja tõstab aktuaatoriga

// override object detection threshold 
#define EI_CLASSIFIER_OBJECT_DETECTION_THRESHOLD 0.5f

#include <a0312_inferencing.h> // edge impulse model
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include <ESP32Servo.h>
#include <Wire.h>
#include <string.h> // for strcmp

// native sensor size
#define RAW_W 320
#define RAW_H 240

// ESP32-S3 EYE / Freenove CAM pins
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5
#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// camera config
static camera_config_t cam_config = {
  .pin_pwdn     = PWDN_GPIO_NUM,
  .pin_reset    = RESET_GPIO_NUM,
  .pin_xclk     = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,
  .pin_d7       = Y9_GPIO_NUM,
  .pin_d6       = Y8_GPIO_NUM,
  .pin_d5       = Y7_GPIO_NUM,
  .pin_d4       = Y6_GPIO_NUM,
  .pin_d3       = Y5_GPIO_NUM,
  .pin_d2       = Y4_GPIO_NUM,
  .pin_d1       = Y3_GPIO_NUM,
  .pin_d0       = Y2_GPIO_NUM,
  .pin_vsync    = VSYNC_GPIO_NUM,
  .pin_href     = HREF_GPIO_NUM,
  .pin_pclk     = PCLK_GPIO_NUM,
  .xclk_freq_hz = 10000000, // 10 MHz 
  .ledc_timer   = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_RGB565, // no JPEG
  .frame_size   = FRAMESIZE_QVGA, // 320×240
  .jpeg_quality = 12,
  .fb_count     = 3,
  .fb_location  = CAMERA_FB_IN_PSRAM,
  .grab_mode    = CAMERA_GRAB_LATEST // drop stale frames
};

// inference buffers (internal RAM)
static uint8_t *infer_buf   = nullptr; // EI input buffer (RGB888)
static uint8_t *rotated_buf = nullptr; // optional rotated buffer
static uint8_t *last_frame  = nullptr; // points to whichever we feed EI

// helpers for RGB565 -> RGB888
static inline uint8_t expand5(uint16_t v) { return (uint8_t)((v * 527 + 23) >> 6); }
static inline uint8_t expand6(uint16_t v) { return (uint8_t)((v * 259 + 33) >> 6); }

//Correct RGB565 byte order from camera.
static void downscale_rgb565_to_rgb888_nn(const uint8_t* src, int srcW, int srcH,
                                          uint8_t* dst, int dstW, int dstH) {
  for (int y = 0; y < dstH; y++) {
    int srcY = (int)((uint32_t)y * (srcH - 1) / (dstH - 1));
    const uint8_t* srcRow = src + srcY * srcW * 2;  // 2 bytes per pixel
    uint8_t* d = dst + y * dstW * 3;                // 3 bytes per pixel RGB888

    for (int x = 0; x < dstW; x++) {
      int srcX = (int)((uint32_t)x * (srcW - 1) / (dstW - 1));
      const uint8_t* px = srcRow + srcX * 2;

      // RGB565 in memory: [high-byte][low-byte]
      uint16_t p = ((uint16_t)px[0] << 8) | (uint16_t)px[1];

      uint8_t r = expand5((p >> 11) & 0x1F);
      uint8_t g = expand6((p >> 5)  & 0x3F);
      uint8_t b = expand5( p        & 0x1F);

      d[0] = r;
      d[1] = g;
      d[2] = b;
      d += 3;
    }
  }
}

// rotate RGB888 buffer 90° (camera mounted sideways on robot)
void rotate90CCW(const uint8_t* src, uint8_t* dst, int srcW, int srcH) {
  for (int y = 0; y < srcH; y++) {
    for (int x = 0; x < srcW; x++) {
      int srcIdx = (y * srcW + x) * 3;
      int dstX   = y;
      int dstY   = (srcW - 1 - x);
      int dstIdx = (dstY * srcH + dstX) * 3;

      dst[dstIdx + 0] = src[srcIdx + 0];
      dst[dstIdx + 1] = src[srcIdx + 1];
      dst[dstIdx + 2] = src[srcIdx + 2];
    }
  }
}

// pack pixels as 0xRRGGBB for Edge Impulse.
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  if (!last_frame) return -1;

  const size_t EI_W = EI_CLASSIFIER_INPUT_WIDTH;
  const size_t EI_H = EI_CLASSIFIER_INPUT_HEIGHT;
  const size_t max_pixels = EI_W * EI_H;
  if (offset >= max_pixels) return -1;
  if (offset + length > max_pixels) length = max_pixels - offset;

  size_t pix = offset * 3; // 3 bytes per pixel in last_frame (RGB888)

  for (size_t i = 0; i < length; i++) {
    uint8_t r = last_frame[pix + 0];
    uint8_t g = last_frame[pix + 1];
    uint8_t b = last_frame[pix + 2];

    out_ptr[i] = (float)((r << 16) | (g << 8) | b);

    pix += 3;
  }
  return 0;
}

// camera init
static bool ei_camera_init() {
  static bool inited = false;
  if (inited) return true;
  if (esp_camera_init(&cam_config) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 1);
  s->set_saturation(s, 0);

  inited = true;
  return true;
}

// DRV8833 motor pins (TT motors)
const int AIN1 = 45; // RIGHT side kollane
const int AIN2 = 46; // RIGHT side roheline
const int BIN1 = 47; // LEFT side  pruun
const int BIN2 = 48; // LEFT side  lilla

// TB6612FNG actuator pins
const int ACT_AIN1 = 40; // TB6612 AIN1
const int ACT_AIN2 = 39; // TB6612 AIN2

// servos
const int SERVO1_PIN = 42; // front servo (unused right now)
const int SERVO2_PIN = 43; // back servo (unused right now)
const int SERVO3_PIN = 41; // big servo (gripper)

// ultrasonic sensor 1 
const int TRIG1_PIN = 1;
const int ECHO1_PIN = 2;

/*
 // ultrasonic sensor 2 (CONFLICT WITH USB! Commented out for testing)
const int TRIG2_PIN = 19;   // USB D+
const int ECHO2_PIN = 20;   // USB D-

float getDistanceCm2() {
  digitalWrite(TRIG2_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG2_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2_PIN, LOW);

  long duration = pulseIn(ECHO2_PIN, HIGH, 30000);
  if (duration == 0) return -1.0;
  return (duration * 0.0343f) / 2.0f;
}
*/

// I2C bus
const int I2C_SDA = 14;
const int I2C_SCL = 21;

// servos 
Servo servo1, servo2, servo3;

// actuator timing
const unsigned long ACT_TRAVEL_MS = 7000; // time to fully lift or fully retract
const unsigned long ACT_PAUSE_MS  = 5000; // (unused now)

// actuator state machine (not used in loop, but helpers are)
enum ActuatorState {
  ACT_MOVING_OUT,
  ACT_HOLDING_OUT,
  ACT_MOVING_IN,
  ACT_HOLDING_IN
};

ActuatorState actState = ACT_MOVING_OUT;
unsigned long actStateStart = 0;

void extendActuator() {
  digitalWrite(ACT_AIN1, HIGH);
  digitalWrite(ACT_AIN2, LOW);
}
void retractActuator() {
  digitalWrite(ACT_AIN1, LOW);
  digitalWrite(ACT_AIN2, HIGH);
}
void stopActuator() {
  digitalWrite(ACT_AIN1, LOW);
  digitalWrite(ACT_AIN2, LOW);
}

// motor helpers
void driveStraight() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

// RIGHT wheel only -> robot nose turns LEFT
void driveSoftLeft() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// LEFT wheel only -> robot nose turns RIGHT
void driveSoftRight() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void stopMotors() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// ultrasonic helpers
float getDistanceCm1() {
  digitalWrite(TRIG1_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG1_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1_PIN, LOW);

  long duration = pulseIn(ECHO1_PIN, HIGH, 30000);
  if (duration == 0) return -1.0;
  return (duration * 0.0343f) / 2.0f;
}

// cup-following logic
enum RobotState { SEARCHING, APPROACHING, STOPPED };
RobotState state = SEARCHING;

// distance to trigger timed final nudge & grab (~4 cm)
const float GRAB_DISTANCE_CM = 4.0f;

const float CUP_CONF  = 0.60f;
const unsigned long CUP_LOST_MS = 800;
unsigned long lastCupSeen = 0;

// scanning behavior
const unsigned long SCAN_INTERVAL_MS = 1500; // stop every 1.5 s to scan
const unsigned long SCAN_SETTLE_MS   = 250;  // let robot settle before picture

// drive pulsing (less aggressive)
const unsigned long DRIVE_CYCLE_MS = 300; // full cycle duration
const unsigned long DRIVE_ON_MS    = 80;  // motors on part of cycle

// final straight nudge time (ms) once we are "close enough"
const unsigned long FINAL_NUDGE_MS = 300;

// cup steering info from last detection
float g_cupOffset = 0.0f;      // -1 .. 1 (left .. right, in EI coords)
bool  g_cupOffsetValid = false;

// did we already grip & lift?
bool g_hasGrabbed = false;

static bool isCupLabel(const char *lbl) {
  if (!lbl) return false;
  return (strcmp(lbl, "measuring_cup") == 0) ||
         (strcmp(lbl, "measuring cup") == 0) ||
         (strcmp(lbl, "cup") == 0);
}

// take one frame, rotate 90° CCW if square, run EI, return whether cup is detected.
bool detectCup() {
  const uint32_t EI_W = EI_CLASSIFIER_INPUT_WIDTH;
  const uint32_t EI_H = EI_CLASSIFIER_INPUT_HEIGHT;

  g_cupOffsetValid = false; // reset each scan

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    return false;
  }

  if (fb->format != PIXFORMAT_RGB565 || fb->width != RAW_W || fb->height != RAW_H) {
    esp_camera_fb_return(fb);
    return false;
  }

  // downscale + RGB565 -> RGB888
  downscale_rgb565_to_rgb888_nn((const uint8_t*)fb->buf, RAW_W, RAW_H,
                                infer_buf, EI_W, EI_H);
  esp_camera_fb_return(fb);

  // If square input (96x96) and rotated_buf exists, rotate 90° CCW
  if (EI_W == EI_H && rotated_buf) {
    rotate90CCW(infer_buf, rotated_buf, EI_W, EI_H);
    last_frame = rotated_buf;
  } else {
    last_frame = infer_buf;
  }

  ei::signal_t signal;
  signal.total_length = EI_W * EI_H;
  signal.get_data     = &ei_camera_get_data;

  ei_impulse_result_t result;
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
  if (err != EI_IMPULSE_OK) {
    return false;
  }

  bool cupDetected = false;

#if (EI_CLASSIFIER_OBJECT_DETECTION == 0)

  if (EI_CLASSIFIER_LABEL_COUNT > 0 && result.classification) {
    float bestConf = 0.0f;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      const auto &c = result.classification[i];
      const char *lbl = c.label ? c.label : "(null)";

      if (isCupLabel(lbl) && c.value >= CUP_CONF && c.value > bestConf) {
        bestConf = c.value;
        cupDetected = true;
        // classification only: no position info
        g_cupOffset = 0.0f;
        g_cupOffsetValid = false;
      }
    }
  }

#else  // EI_CLASSIFIER_OBJECT_DETECTION == 1

  if (result.bounding_boxes_count > 0 && result.bounding_boxes) {
    float bestConf = 0.0f;

    for (size_t i = 0; i < result.bounding_boxes_count; i++) {
      const auto &bb = result.bounding_boxes[i];
      if (bb.value == 0) continue; 

      const char *lbl = bb.label ? bb.label : "(null)";

      if (isCupLabel(lbl) && bb.value >= CUP_CONF && bb.value > bestConf) {
        bestConf = bb.value;
        cupDetected = true;

        // compute normalized horizontal offset in [-1, 1]
        float centerX = bb.x + bb.width / 2.0f;
        float halfW   = (float)EI_W / 2.0f;
        g_cupOffset   = (centerX - halfW) / halfW;  // EI coords: left=-1, right=+1
        g_cupOffsetValid = true;
      }
    }
  }

#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  (void)result.anomaly; // not used
#endif

  if (cupDetected) {
    lastCupSeen = millis();
  }

  return cupDetected;
}

// SETUP
void setup() {
  Serial.begin(115200);

  // camera init
  ei_camera_init();

  const uint32_t EI_W = EI_CLASSIFIER_INPUT_WIDTH;
  const uint32_t EI_H = EI_CLASSIFIER_INPUT_HEIGHT;

  infer_buf   = (uint8_t*) heap_caps_malloc(EI_W * EI_H * 3, MALLOC_CAP_8BIT);
  rotated_buf = (uint8_t*) heap_caps_malloc(EI_W * EI_H * 3, MALLOC_CAP_8BIT);

  Wire.begin(I2C_SDA, I2C_SCL);

  // motor pins
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  stopMotors();

  // actuator pins
  pinMode(ACT_AIN1, OUTPUT);
  pinMode(ACT_AIN2, OUTPUT);
  stopActuator();

  // actively retract actuator on startup so we KNOW it's fully down/in
  retractActuator();
  delay(ACT_TRAVEL_MS);   // run long enough to hit full bottom
  stopActuator();

  // ultrasonic 1 pins
  pinMode(TRIG1_PIN, OUTPUT); 
  pinMode(ECHO1_PIN, INPUT);

  // ultrasonic 2 pins
  /*
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO2_PIN, INPUT);
  */

  // servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);

  // big servo starts open at 90°
  servo3.write(90);

  // we start in SEARCHING, no cup grabbed yet
  state = SEARCHING;
  g_hasGrabbed = false;

  delay(200);
}

// LOOP
void loop() {
  static unsigned long lastScanMs = 0;
  unsigned long now = millis();

  bool cupDetected = false;

  switch (state) {
    case SEARCHING:
      stopMotors();
      cupDetected = detectCup();

      if (cupDetected) {
        state = APPROACHING;
        lastCupSeen = millis();
        lastScanMs = now;
        g_hasGrabbed = false;    // new grab sequence
      }
      break;

    case APPROACHING: {
      // periodically stop and rescan with the camera
      if (now - lastScanMs >= SCAN_INTERVAL_MS) {
        stopMotors();
        delay(SCAN_SETTLE_MS);

        cupDetected = detectCup();
        lastScanMs = millis();

        if (!cupDetected && (millis() - lastCupSeen > CUP_LOST_MS)) {
          stopMotors();
          state = SEARCHING;
          break;
        }
      }

      float d = getDistanceCm1(); // use front ultrasonic

      // close enough to start timed final nudge & grab (~4 cm)
      if (d > 0 && d <= GRAB_DISTANCE_CM) {
        stopMotors();
        state = STOPPED;
      } else {
        // drive in short pulses and steer towards cup based on g_cupOffset
        unsigned long phase = now % DRIVE_CYCLE_MS;
        if (phase < DRIVE_ON_MS) {
          float offset = g_cupOffsetValid ? g_cupOffset : 0.0f;
          const float DEADZONE = 0.15f; // don't over-steer around center

          // camera orientation requires flipped steering
          if (offset < -DEADZONE) {
            driveSoftRight();   // cup looks left -> turn right
          } else if (offset > DEADZONE) {
            driveSoftLeft();    // cup looks right -> turn left
          } else {
            driveStraight();
          }
        } else {
          stopMotors();
        }
      }
      break;
    }

    case STOPPED:
      stopMotors();

      // one-time FINAL NUDGE + grip + lift
      if (!g_hasGrabbed) {
        // 1) small timed straight nudge closer to the cup
        driveStraight();
        delay(FINAL_NUDGE_MS);
        stopMotors();

        // 2) close big servo around the cup
        servo3.write(137);      // grip
        delay(1000);            // give servo time

        // 3) lift actuator fully OUT (up)
        extendActuator();
        delay(ACT_TRAVEL_MS);   // blocking lift
        stopActuator();

        g_hasGrabbed = true;
      }

      // after grabbing, just stay stopped
      break;
  }

  delay(15);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid Edge Impulse sensor type"
#endif
