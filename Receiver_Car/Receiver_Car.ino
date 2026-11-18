/*
  ============================================================================
   Receiver_Car.ino  (ESP32 DevKit V1)  -  Smart 3-in-1 Robot Car
  ============================================================================
   Hardware : ESP32 DevKit V1 (30-pin)
              L298N dual H-bridge + 4 DC gear motors (4WD)
              HC-SR04 ultrasonic distance sensor (powered at 3.3 V)
              2 x 18650 (7.4 V) + power switch

   Wiring   (ESP32 silkscreen labels):
     L298N:  IN1 -> D18 (GPIO18)   IN2 -> D19 (GPIO19)
             IN3 -> D23 (GPIO23)   IN4 -> D14 (GPIO14)
             ENA -> D25 (GPIO25)   ENB -> D26 (GPIO26)   [PWM via LEDC]
     HC-SR04: TRIG -> D27 (GPIO27)   ECHO -> D32 (GPIO32)  (VCC = 3.3 V)

   Modes    : 0 = GESTURE      directions arrive over ESP-NOW
              1 = MOBILE WEB   SoftAP "Smart_Car_AP" + dashboard at 192.168.4.1
              2 = FOLLOW ME    autonomous ultrasonic tracking

   Follow   : < 15 cm  -> reverse
              15..30 cm -> hold position
              30..60 cm -> drive forward
              > 60 cm  -> stop (target lost)

   Safety   : radio timeout stops the car, web command timeout stops the car,
              mode changes always stop the car, every direction change ramps
              through zero, and an obstacle < 15 cm blocks FORWARD in Gesture
              and Mobile Web modes (Back/Left/Right still work).

   NOTE     : GPIO0 (D3) is NOT used on the receiver - the mode button lives
              on the transmitter (glove) only.
  ============================================================================
*/

#include <WiFi.h>
#include <esp_now.h>
#include <WebServer.h>
#include "WebPage.h"

// ============================================================================
//  CONFIGURATION
// ============================================================================
#define FW_VERSION        "2.0-ESP32"
#define PROTOCOL_VERSION  2
#define DEBUG_SERIAL      1

// ---- Motor driver pins (ESP32 GPIO) ----
#define PIN_IN1   18    // D18
#define PIN_IN2   19    // D19
#define PIN_IN3   23    // D23
#define PIN_IN4   14    // D14
#define PIN_ENA   25    // D25  PWM (LEDC ch0)
#define PIN_ENB   26    // D26  PWM (LEDC ch1)

// ---- Ultrasonic pins ----
#define PIN_TRIG  27    // D27
#define PIN_ECHO  32    // D32

// ---- Speed (LEDC 10-bit: 0..1023) ----
#define SPEED_STRAIGHT   780
#define SPEED_TURN       700
#define SPEED_FOLLOW     520
#define SLEW_STEP        110

// ---- Timing (ms) ----
#define MOTOR_TICK_MS      20
#define ULTRA_INTERVAL_MS  60
#define ACK_INTERVAL_MS    100
#define RADIO_TIMEOUT_MS   600
#define WEB_TIMEOUT_MS     700
#define DIST_STALE_MS      400

// ---- Follow distances (cm) ----
#define FOLLOW_NEAR_CM     15.0f
#define FOLLOW_FAR_CM      30.0f
#define FOLLOW_MAX_CM      60.0f
#define FOLLOW_HYST_CM      3.0f
#define DIST_INVALID       255

// ---- SoftAP ----
#define AP_SSID   "Smart_Car_AP"
#define AP_PASS   "12345678"
#define AP_CHANNEL 1

// ---- LEDC ----
#define ENA_CH  0
#define ENB_CH  1

// ============================================================================
//  TYPES
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

// ============================================================================
//  STATE
// ============================================================================
volatile uint8_t gMode     = MODE_GESTURE;
volatile uint8_t gRadioDir = DIR_STOP;
volatile bool    gFreshCmd = false;

uint8_t  activeMode   = MODE_GESTURE;
uint8_t  commandDir   = DIR_STOP;
uint8_t  rampedDir    = DIR_STOP;
uint8_t  followCmd    = DIR_STOP;

int16_t  speedL = 0, speedR = 0;
int16_t  targetSpeed = SPEED_STRAIGHT;

unsigned long lastRadioMs   = 0;
unsigned long lastWebCmdMs  = 0;
unsigned long lastMotorMs   = 0;
unsigned long lastUltraMs   = 0;
unsigned long lastAckMs     = 0;
unsigned long lastDistMs    = 0;

bool     radioSeen    = false;
bool     webActive    = false;

uint8_t  ackDest[6];
bool     ackPending   = false;
bool     ackPeerReady = false;

float    distFiltered = -1.0f;
uint8_t  distReport   = DIST_INVALID;

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

const char* modeName(uint8_t m) {
  switch (m) {
    case MODE_GESTURE:    return "GESTURE";
    case MODE_MOBILE_WEB: return "MOBILE WEB";
    case MODE_FOLLOW_ME:  return "FOLLOW ME";
    default:              return "UNKNOWN";
  }
}
static inline int16_t slewTo(int16_t cur, int16_t tgt, int16_t step) {
  if (cur < tgt) return (cur + step > tgt) ? tgt : cur + step;
  if (cur > tgt) return (cur - step < tgt) ? tgt : cur - step;
  return cur;
}

// ============================================================================
//  MOTOR DRIVER (LEDC PWM)
// ============================================================================
void applyPins(uint8_t dir, int16_t pwm) {
  switch (dir) {
    case DIR_FORWARD:
      digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
      digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
      break;
    case DIR_BACKWARD:
      digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
      digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
      break;
    case DIR_LEFT:
      digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH);
      digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW);
      break;
    case DIR_RIGHT:
      digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW);
      digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH);
      break;
    default:
      digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, LOW);
      digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, LOW);
      pwm = 0;
      break;
  }
  ledcWrite(ENA_CH, constrain(pwm, 0, 1023));
  ledcWrite(ENB_CH, constrain(pwm, 0, 1023));
}
void motorsHardStop() {
  commandDir = DIR_STOP;
  rampedDir  = DIR_STOP;
  speedL = speedR = 0;
  applyPins(DIR_STOP, 0);
}
void motorsUpdate() {
  if (commandDir != rampedDir) {
    if (speedL > 0 || speedR > 0) {
      speedL = slewTo(speedL, 0, SLEW_STEP * 2);
      speedR = slewTo(speedR, 0, SLEW_STEP * 2);
      applyPins(rampedDir, (speedL + speedR) / 2);
      return;
    }
    rampedDir = (commandDir == DIR_STOP) ? DIR_STOP : commandDir;
  }
  int16_t tgt = 0;
  switch (rampedDir) {
    case DIR_FORWARD:
    case DIR_BACKWARD: tgt = targetSpeed; break;
    case DIR_LEFT:
    case DIR_RIGHT:    tgt = SPEED_TURN;  break;
    default:           tgt = 0;           break;
  }
  speedL = slewTo(speedL, tgt, SLEW_STEP);
  speedR = slewTo(speedR, tgt, SLEW_STEP);
  applyPins(rampedDir, (speedL + speedR) / 2);
}

// ============================================================================
//  ULTRASONIC
// ============================================================================
float measureDistance() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (us == 0) return -1.0f;
  float cm = (us * 0.0343f) / 2.0f;
  if (cm < 2.0f || cm > 400.0f) return -1.0f;
  return cm;
}
void serviceUltrasonic() {
  float raw = measureDistance();
  if (raw < 0) {
    if (millis() - lastDistMs > DIST_STALE_MS) { distFiltered = -1.0f; distReport = DIST_INVALID; }
    return;
  }
  distFiltered = (distFiltered < 0) ? raw : (0.6f * distFiltered + 0.4f * raw);
  lastDistMs = millis();
  distReport = (uint8_t)constrain(distFiltered, 0.0f, 254.0f);
#if DEBUG_SERIAL
  Serial.print(F("ULTRA dist=")); Serial.println(distFiltered, 1);
#endif
}
bool forwardBlocked() {
  return (distFiltered > 0.0f && distFiltered < 15.0f);
}

// ============================================================================
//  MODE LOGIC
// ============================================================================
void computeFollowCommand() {
  if (distFiltered < 0) { followCmd = DIR_STOP; commandDir = DIR_STOP; return; }
  float d = distFiltered;
  switch (followCmd) {
    case DIR_FORWARD:
      if (d <= FOLLOW_FAR_CM || d > FOLLOW_MAX_CM) followCmd = DIR_STOP;
      break;
    case DIR_BACKWARD:
      if (d >= FOLLOW_NEAR_CM) followCmd = DIR_STOP;
      break;
    default:
      if (d > FOLLOW_FAR_CM + FOLLOW_HYST_CM && d <= FOLLOW_MAX_CM) followCmd = DIR_FORWARD;
      else if (d < FOLLOW_NEAR_CM - FOLLOW_HYST_CM) followCmd = DIR_BACKWARD;
      else followCmd = DIR_STOP;
      break;
  }
  commandDir = followCmd;
}

// ============================================================================
//  ESP-NOW
// ============================================================================
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(ControlPacket)) return;
  ControlPacket pkt;
  memcpy(&pkt, incomingData, sizeof(pkt));
  if (pkt.ver != PROTOCOL_VERSION) return;
  gMode     = (pkt.mode < MODE_COUNT) ? pkt.mode : MODE_GESTURE;
  gRadioDir = (pkt.direction <= DIR_RIGHT) ? pkt.direction : DIR_STOP;
  gFreshCmd = true;
  lastRadioMs = millis();
  radioSeen   = true;
  if (memcmp(ackDest, mac, 6) != 0) { memcpy(ackDest, mac, 6); ackPeerReady = false; }
  ackPending = true;
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr; (void)status;
}
void sendAck() {
  if (!ackPeerReady) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, ackDest, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) == ESP_OK) ackPeerReady = true;
    else return;
  }
  AckPacket ack;
  ack.ver = PROTOCOL_VERSION;
  ack.mode = activeMode;
  ack.distanceCm = distReport;
  ack.flags = (speedL > 30) ? 0x01 : 0x00;
  esp_now_send(ackDest, (uint8_t *)&ack, sizeof(ack));
}

// ============================================================================
//  WEB DASHBOARD
// ============================================================================
IPAddress apIP(192, 168, 4, 1);
IPAddress apMask(255, 255, 255, 0);
WebServer server(80);

#include "WebPage.h"

void handleRoot() { server.send_P(200, "text/html", PAGE_HTML); }
void handleCommand(uint8_t dir) {
  if (activeMode != MODE_MOBILE_WEB) { server.send(409, "text/plain", "NOT_IN_WEB_MODE"); return; }
  if (dir == DIR_FORWARD && forwardBlocked()) {
    commandDir = DIR_STOP; webActive = false;
    server.send(409, "text/plain", "OBSTACLE"); return;
  }
  commandDir = dir;
  webActive = (dir != DIR_STOP);
  lastWebCmdMs = millis();
  server.send(200, "text/plain", "OK");
}
void handleForward()  { handleCommand(DIR_FORWARD);  }
void handleBackward() { handleCommand(DIR_BACKWARD); }
void handleLeft()     { handleCommand(DIR_LEFT);     }
void handleRight()    { handleCommand(DIR_RIGHT);    }
void handleStop()     { handleCommand(DIR_STOP);     }
void handleStatus() {
  char buf[160];
  int distOut = (distReport == DIST_INVALID) ? -1 : (int)distReport;
  snprintf(buf, sizeof(buf),
           "{\"mode\":%u,\"modeName\":\"%s\",\"dist\":%d,\"moving\":%s,\"radio\":%s,\"obstacle\":%s}",
           (unsigned)activeMode, modeName(activeMode), distOut,
           (speedL > 30) ? "true" : "false",
           radioSeen ? "true" : "false",
           forwardBlocked() ? "true" : "false");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}
void handleNotFound() { server.send(404, "text/plain", "Not found"); }

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  DBGLN();
  DBGLN(F("=== SMART CAR RX (ESP32) ==="));

  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  ledcSetup(ENA_CH, 1000, 10);
  ledcAttachPin(PIN_ENA, ENA_CH);
  ledcSetup(ENB_CH, 1000, 10);
  ledcAttachPin(PIN_ENB, ENB_CH);
  motorsHardStop();

  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect();
  WiFi.softAPConfig(apIP, apIP, apMask);
  if (WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4)) {
    DBG(F("SoftAP up: ")); DBGLN(AP_SSID);
    DBG(F("IP: "));        DBGLN(WiFi.softAPIP());
  } else {
    DBGLN(F("SoftAP FAILED"));
  }

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
    DBGLN(F("ESP-NOW receiver ready"));
  } else {
    DBGLN(F("ESP-NOW init failed"));
  }

  server.on("/",         HTTP_GET, handleRoot);
  server.on("/forward",  HTTP_GET, handleForward);
  server.on("/backward", HTTP_GET, handleBackward);
  server.on("/left",     HTTP_GET, handleLeft);
  server.on("/right",    HTTP_GET, handleRight);
  server.on("/stop",     HTTP_GET, handleStop);
  server.on("/status",   HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
  DBGLN(F("Web server started"));

  lastRadioMs = millis() - RADIO_TIMEOUT_MS - 1;
  DBGLN(F("=== READY ==="));
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  if (gFreshCmd) {
    gFreshCmd = false;
    if (gMode != activeMode) {
      activeMode = gMode;
      motorsHardStop();
      webActive = false;
      followCmd = DIR_STOP;
      distFiltered = -1.0f;
      distReport = DIST_INVALID;
      DBG(F("Mode -> ")); DBGLN(modeName(activeMode));
    }
  }

  bool radioAlive = radioSeen && (now - lastRadioMs < RADIO_TIMEOUT_MS);

  // service ultrasonic in every mode
  if (now - lastUltraMs >= ULTRA_INTERVAL_MS) {
    lastUltraMs = now;
    serviceUltrasonic();
  }

  switch (activeMode) {
    case MODE_GESTURE:
      if (gRadioDir == DIR_FORWARD && (now - lastUltraMs > 30)) {
        lastUltraMs = now;
        serviceUltrasonic();
      }
      commandDir = radioAlive ? gRadioDir : DIR_STOP;
      if (commandDir == DIR_FORWARD && forwardBlocked()) {
        commandDir = DIR_STOP;
        speedL = 0; speedR = 0; rampedDir = DIR_STOP;
        applyPins(DIR_STOP, 0);
#if DEBUG_SERIAL
        Serial.print(F("GESTURE forward blocked, dist="));
        Serial.println(distFiltered, 1);
#endif
      }
      targetSpeed = SPEED_STRAIGHT;
      break;
    case MODE_MOBILE_WEB:
      if (webActive && (now - lastWebCmdMs > WEB_TIMEOUT_MS)) {
        webActive = false; commandDir = DIR_STOP;
      }
      if (commandDir == DIR_FORWARD && forwardBlocked()) {
        commandDir = DIR_STOP;
        speedL = 0; speedR = 0; rampedDir = DIR_STOP;
        applyPins(DIR_STOP, 0);
      }
      targetSpeed = SPEED_STRAIGHT;
      break;
    case MODE_FOLLOW_ME:
      computeFollowCommand();
      targetSpeed = SPEED_FOLLOW;
      break;
    default:
      commandDir = DIR_STOP;
      break;
  }

  if (now - lastMotorMs >= MOTOR_TICK_MS) {
    lastMotorMs = now;
    motorsUpdate();
  }

  if (ackPending && (now - lastAckMs >= ACK_INTERVAL_MS)) {
    ackPending = false;
    lastAckMs = now;
    sendAck();
  }

  server.handleClient();
}
