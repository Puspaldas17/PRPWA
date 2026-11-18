/*
  ============================================================================
   Transmitter_Glove.ino  (ESP32 DevKit V1)  -  Smart 3-in-1 Robot Car Remote
  ============================================================================
   Hardware : ESP32 DevKit V1 (30-pin)
              MPU6050 accelerometer (I2C)
              SSD1306 0.96" OLED 128x64 (I2C)
              Tactile push button (mode selector)

   Wiring   : SDA  -> D21 (GPIO21)  shared MPU6050 + OLED
              SCL  -> D22 (GPIO22)  shared MPU6050 + OLED
              BTN  -> D4  (GPIO4)   other leg to GND (internal pull-up)
                      (GPIO4 is a safe GPIO - no strapping/boot restrictions)

   Modes    : 0 = GESTURE      tilt the glove to drive
              1 = MOBILE WEB   car hosts its own Wi-Fi dashboard
              2 = FOLLOW ME    car tracks a target with HC-SR04

   Link     : ESP-NOW broadcast every 50 ms  ->  Receiver answers with an ACK
              packet carrying its live mode + measured distance.

   Libraries: Adafruit GFX Library, Adafruit SSD1306
              (MPU6050 is driven by direct I2C register access - no lib needed)

   Notes    : 100% non-blocking. Everything is scheduled from millis().
              Left/Right gesture mapping is corrected (right tilt -> right).
  ============================================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ============================================================================
//  COMPILE-TIME CONFIGURATION
// ============================================================================
#define FW_VERSION        "2.0-ESP32"
#define PROTOCOL_VERSION  2

#define DEBUG_SERIAL      1

#define INVERT_PITCH      false
#define INVERT_ROLL       false

#define TILT_ENTER_DEG    18.0f
#define TILT_EXIT_DEG     10.0f
#define TILT_FILTER_ALPHA 0.30f

#define TX_INTERVAL_MS    50
#define LINK_TIMEOUT_MS   1500
#define OLED_REFRESH_MS   100
#define SPLASH_MS         2600
#define BTN_DEBOUNCE_MS   30

// ============================================================================
//  PINS  (ESP32 DevKit V1 silkscreen labels)
// ============================================================================
static const int PIN_SDA    = 21;   // D21
static const int PIN_SCL    = 22;   // D22
static const int PIN_BUTTON = 4;    // D4  (GPIO4)

// ============================================================================
//  OLED
// ============================================================================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_ADDR      0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool oledOK = false;

// ============================================================================
//  MPU6050 (direct register access)
// ============================================================================
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t REG_SMPLRT_DIV   = 0x19;
static const uint8_t REG_CONFIG       = 0x1A;
static const uint8_t REG_GYRO_CONFIG  = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_WHO_AM_I     = 0x75;

bool    mpuOK = false;
int16_t rawAx = 0, rawAy = 0, rawAz = 0;
float   pitchDeg = 0.0f;
float   rollDeg  = 0.0f;

// ============================================================================
//  MODES / DIRECTIONS
// ============================================================================
enum CarMode : uint8_t {
  MODE_GESTURE    = 0,
  MODE_MOBILE_WEB = 1,
  MODE_FOLLOW_ME  = 2,
  MODE_COUNT      = 3
};
enum DriveDir : uint8_t {
  DIR_STOP     = 0,
  DIR_FORWARD  = 1,
  DIR_BACKWARD = 2,
  DIR_LEFT     = 3,
  DIR_RIGHT    = 4
};

volatile uint8_t currentMode = MODE_GESTURE;
uint8_t          currentDir  = DIR_STOP;

// ============================================================================
//  ESP-NOW PACKETS
// ============================================================================
struct __attribute__((packed)) ControlPacket {
  uint8_t ver;
  uint8_t mode;
  uint8_t direction;
  uint8_t seq;
  int16_t pitch10;
  int16_t roll10;
};
struct __attribute__((packed)) AckPacket {
  uint8_t ver;
  uint8_t mode;
  uint8_t distanceCm;
  uint8_t flags;
};

uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

ControlPacket txPacket;
AckPacket     lastAck;

bool          ackValid = false;
bool          linkUp   = false;
unsigned long lastAckMs = 0;
unsigned long lastTxMs  = 0;
unsigned long lastOledMs = 0;
unsigned long splashStartMs = 0;
unsigned long lastSplashMs = 0;
unsigned long lastBtnMs = 0;
bool          splashDone = false;

uint8_t  seqCounter = 0;
bool     btnStable = HIGH;
bool     btnLastRead = HIGH;

// ============================================================================
//  HELPERS
// ============================================================================
#if DEBUG_SERIAL
  #define DBG(x)    Serial.print(x)
  #define DBGLN(x)  Serial.println(x)
#else
  #define DBG(x)
  #define DBGLN(x)
#endif

static inline void i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}
static bool i2cRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}
const char* modeName(uint8_t m) {
  switch (m) {
    case MODE_GESTURE:    return "GESTURE";
    case MODE_MOBILE_WEB: return "MOBILE WEB";
    case MODE_FOLLOW_ME:  return "FOLLOW ME";
    default:              return "UNKNOWN";
  }
}
const char* dirName(uint8_t d) {
  switch (d) {
    case DIR_FORWARD:  return "FWD";
    case DIR_BACKWARD: return "BACK";
    case DIR_LEFT:     return "LEFT";
    case DIR_RIGHT:    return "RIGHT";
    default:           return "STOP";
  }
}

// ============================================================================
//  MPU6050
// ============================================================================
bool mpuInit() {
  uint8_t id = 0;
  if (!i2cRead(REG_WHO_AM_I, &id, 1)) return false;
  if (id == 0x00 || id == 0xFF) return false;
  i2cWrite(REG_PWR_MGMT_1, 0x00);
  delay(50);
  i2cWrite(REG_PWR_MGMT_1, 0x01);
  i2cWrite(REG_SMPLRT_DIV, 4);
  i2cWrite(REG_CONFIG, 3);
  i2cWrite(REG_GYRO_CONFIG, 0x00);
  i2cWrite(REG_ACCEL_CONFIG, 0x00);
  delay(20);
  return true;
}
void mpuRead() {
  uint8_t buf[6];
  if (!i2cRead(REG_ACCEL_XOUT_H, buf, 6)) { mpuOK = false; return; }
  rawAx = (int16_t)((buf[0] << 8) | buf[1]);
  rawAy = (int16_t)((buf[2] << 8) | buf[3]);
  rawAz = (int16_t)((buf[4] << 8) | buf[5]);
  float ax = rawAx / 16384.0f;
  float ay = rawAy / 16384.0f;
  float az = rawAz / 16384.0f;
  float p = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
  float r = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  if (INVERT_PITCH) p = -p;
  if (INVERT_ROLL)  r = -r;
  pitchDeg += TILT_FILTER_ALPHA * (p - pitchDeg);
  rollDeg  += TILT_FILTER_ALPHA * (r - rollDeg);
}

// ============================================================================
//  GESTURE (left/right corrected: right tilt -> right)
// ============================================================================
uint8_t gestureDir = DIR_STOP;
void updateGesture() {
  if (!mpuOK) { gestureDir = DIR_STOP; return; }
  float p = pitchDeg, r = rollDeg;
  if (gestureDir != DIR_STOP) {
    bool keep = false;
    switch (gestureDir) {
      case DIR_FORWARD:  keep = (p >  TILT_EXIT_DEG); break;
      case DIR_BACKWARD: keep = (p < -TILT_EXIT_DEG); break;
      case DIR_RIGHT:    keep = (r < -TILT_EXIT_DEG); break;   // corrected
      case DIR_LEFT:     keep = (r >  TILT_EXIT_DEG); break;   // corrected
    }
    if (!keep) gestureDir = DIR_STOP;
  }
  if (gestureDir == DIR_STOP) {
    bool pitchDominant = fabsf(p) >= fabsf(r);
    if (pitchDominant) {
      if      (p >  TILT_ENTER_DEG) gestureDir = DIR_FORWARD;
      else if (p < -TILT_ENTER_DEG) gestureDir = DIR_BACKWARD;
    } else {
      if      (r >  TILT_ENTER_DEG) gestureDir = DIR_LEFT;     // corrected
      else if (r < -TILT_ENTER_DEG) gestureDir = DIR_RIGHT;    // corrected
    }
  }
}

// ============================================================================
//  BUTTON
// ============================================================================
void handleButton(unsigned long now) {
  bool reading = digitalRead(PIN_BUTTON);
  if (reading != btnLastRead) { lastBtnMs = now; btnLastRead = reading; }
  if ((now - lastBtnMs) > BTN_DEBOUNCE_MS && reading != btnStable) {
    btnStable = reading;
    if (btnStable == LOW) {
      currentMode = (currentMode + 1) % MODE_COUNT;
      currentDir = DIR_STOP;
      gestureDir = DIR_STOP;
      DBGLN(modeName(currentMode));
    }
  }
}

// ============================================================================
//  ESP-NOW
// ============================================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr; (void)status;
}
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(AckPacket)) return;
  AckPacket tmp;
  memcpy(&tmp, incomingData, sizeof(tmp));
  if (tmp.ver != PROTOCOL_VERSION) return;
  lastAck = tmp;
  ackValid = true;
  lastAckMs = millis();
}

bool espNowBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { DBGLN("ESP-NOW init failed"); return false; }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { DBGLN("add peer failed"); return false; }
  return true;
}
void sendPacket() {
  txPacket.ver = PROTOCOL_VERSION;
  txPacket.mode = currentMode;
  txPacket.direction = currentDir;
  txPacket.seq = seqCounter++;
  txPacket.pitch10 = (int16_t)constrain(pitchDeg * 10.0f, -32767.0f, 32767.0f);
  txPacket.roll10  = (int16_t)constrain(rollDeg  * 10.0f, -32767.0f, 32767.0f);
  esp_now_send(broadcastAddr, (uint8_t *)&txPacket, sizeof(txPacket));
}

// ============================================================================
//  OLED
// ============================================================================
void drawStatusBar() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
  if (linkUp) display.fillCircle(4, 5, 2, SSD1306_WHITE);
  else        display.drawCircle(4, 5, 2, SSD1306_WHITE);
  display.setCursor(10, 1);
  display.print(linkUp ? F("LINK OK") : F("NO LINK"));
  display.setCursor(104, 1);
  display.print(F("M")); display.print((int)currentMode + 1); display.print(F("/3"));
}
void drawIconGesture(int cx, int cy) {
  display.fillRoundRect(cx - 9, cy - 2, 18, 13, 4, SSD1306_WHITE);
  for (int i = 0; i < 4; i++) display.fillRoundRect(cx - 9 + i * 5, cy - 11, 3, 11, 1, SSD1306_WHITE);
  display.fillRect(cx - 4, cy + 11, 8, 4, SSD1306_WHITE);
}
void drawIconWifi(int cx, int cy) {
  display.drawCircle(cx, cy + 4, 14, SSD1306_WHITE);
  display.drawCircle(cx, cy + 4, 8,  SSD1306_WHITE);
  display.fillCircle(cx, cy + 4, 2,  SSD1306_WHITE);
  display.fillRect(cx - 16, cy + 5, 33, 12, SSD1306_BLACK);
}
void drawIconRobot(int cx, int cy) {
  display.drawRoundRect(cx - 15, cy - 8, 30, 21, 5, SSD1306_WHITE);
  display.fillCircle(cx - 6, cy + 1, 3, SSD1306_WHITE);
  display.fillCircle(cx + 6, cy + 1, 3, SSD1306_WHITE);
  display.drawLine(cx, cy - 8, cx, cy - 14, SSD1306_WHITE);
  display.fillCircle(cx, cy - 16, 2, SSD1306_WHITE);
  display.drawLine(cx - 5, cy + 8, cx + 5, cy + 8, SSD1306_WHITE);
}
void drawSplash(unsigned long now) {
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.drawRect(2, 2, 124, 60, SSD1306_WHITE);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(26, 12); display.print(F("SMART CAR v1.0"));
  display.setCursor(16, 28); display.print(F("Made by Jagadish Sahoo"));
  display.drawRect(14, 44, 100, 8, SSD1306_WHITE);
  float p = constrain((float)(now - splashStartMs) / (float)SPLASH_MS, 0.0f, 1.0f);
  display.fillRect(16, 46, (int)(96 * p), 4, SSD1306_WHITE);
  display.display();
}
void drawMainScreen() {
  display.clearDisplay();
  drawStatusBar();
  int iconCx = 22, iconCy = 30;
  switch (currentMode) {
    case MODE_GESTURE:    drawIconGesture(iconCx, iconCy); break;
    case MODE_MOBILE_WEB: drawIconWifi(iconCx, iconCy);    break;
    case MODE_FOLLOW_ME:  drawIconRobot(iconCx, iconCy);   break;
  }
  display.setTextSize(1);
  display.setCursor(46, 15); display.print(F("MODE:"));
  display.setCursor(46, 25); display.print(modeName(currentMode));
  display.drawLine(0, 44, 127, 44, SSD1306_WHITE);
  switch (currentMode) {
    case MODE_GESTURE:
      if (!mpuOK) { display.setCursor(2, 49); display.print(F("!! MPU6050 ERROR !!")); }
      else {
        display.setCursor(2, 49);
        display.print(F("P:")); if (pitchDeg >= 0) display.print('+'); display.print(pitchDeg, 1);
        display.setCursor(66, 49);
        display.print(F("R:")); if (rollDeg >= 0) display.print('+'); display.print(rollDeg, 1);
        display.setCursor(2, 57); display.print(F("DIR: ")); display.print(dirName(currentDir));
      }
      break;
    case MODE_MOBILE_WEB:
      display.setCursor(2, 48); display.print(F("AP: Smart_Car_AP"));
      display.setCursor(2, 57); display.print(F("http://192.168.4.1"));
      break;
    case MODE_FOLLOW_ME:
      display.setCursor(2, 48); display.print(F("Ultrasonic tracking"));
      display.setCursor(2, 57); display.print(F("DIST: "));
      if (ackValid && lastAck.distanceCm != 255) { display.print(lastAck.distanceCm); display.print(F(" cm")); }
      else display.print(F("-- cm"));
      break;
  }
  display.display();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  DBGLN();
  DBGLN(F("=== SMART CAR TX (GLOVE) ESP32 ==="));

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  btnStable = digitalRead(PIN_BUTTON);
  btnLastRead = btnStable;

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) { display.setRotation(0); display.clearDisplay(); display.display(); }
  else DBGLN(F("OLED not found (expected 0x3C)"));

  splashStartMs = millis();
  splashDone = false;
  if (oledOK) drawSplash(splashStartMs);

  mpuOK = mpuInit();
  DBG(F("MPU6050: ")); DBGLN(mpuOK ? F("OK") : F("NOT FOUND"));
  if (mpuOK) { for (int i = 0; i < 5; i++) { mpuRead(); delay(10); } }

  if (!espNowBegin()) DBGLN(F("ESP-NOW unavailable"));

  txPacket.ver = PROTOCOL_VERSION;
  txPacket.mode = currentMode;
  txPacket.direction = DIR_STOP;

  DBGLN(F("=== READY ==="));
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  if (!splashDone) {
    if (now - splashStartMs >= SPLASH_MS) {
      splashDone = true;
      if (oledOK) display.clearDisplay();
    } else {
      if (oledOK && (now - lastSplashMs >= 60)) { lastSplashMs = now; drawSplash(now); }
      return;
    }
  }

  handleButton(now);
  if (mpuOK) mpuRead();

  if (currentMode == MODE_GESTURE) { updateGesture(); currentDir = gestureDir; }
  else { gestureDir = DIR_STOP; currentDir = DIR_STOP; }

  if (now - lastTxMs >= TX_INTERVAL_MS) { lastTxMs = now; sendPacket(); }

  linkUp = ackValid && (now - lastAckMs < LINK_TIMEOUT_MS);

  if (oledOK && (now - lastOledMs >= OLED_REFRESH_MS)) {
    lastOledMs = now;
    drawMainScreen();
  }
}
