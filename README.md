# Smart 3-in-1 Robot Car — firmware package

## Repository layout

```
PRPWA/
├── README.md
├── OVERVIEW.md                          ← Visual overview: block/circuit diagrams, BOM, flowcharts
├── platformio.ini                       ← PlatformIO dual-env build config
├── .vscode/                             ← VS Code IntelliSense + recommended extensions
│
├── Receiver_Car/                        ← Unit #2 — Car receiver sketch
│   ├── Receiver_Car.ino
│   └── WebPage.h                        ← Mobile dashboard (must stay next to .ino)
│
├── Transmitter_Glove/                   ← Unit #1 — Glove transmitter sketch
│   └── Transmitter_Glove.ino
│
├── ESP32S3_CAM/                         ← Unit #3 — FPV camera sketch (ESP32-S3)
│   ├── ESP32S3_CAM_AP.ino              ← Main sketch (SoftAP + camera server)
│   ├── app_httpd.cpp                    ← HTTP camera stream server
│   ├── board_config.h                   ← Camera model selector
│   ├── camera_pins.h                    ← GPIO pin definitions per board
│   ├── camera_index.h                   ← Embedded web UI (PROGMEM)
│   ├── partitions.csv                   ← Custom partition table (3 MB app space)
│   ├── ci.yml                           ← Arduino CI build matrix
│   ├── ESP32S3_CAM_AP.rar              ← Original archive (can be deleted after extract)
│   └── .vscode/
│       └── c_cpp_properties.json        ← IntelliSense for ESP32-S3
│
├── firmware/                            ← All ready-to-flash binaries
│   ├── Receiver_Car.bin                 ← Car receiver  (flash @ 0x0)
│   ├── Transmitter_Glove.bin            ← Glove TX      (flash @ 0x0)
│   ├── ESP32S3_CAM_AP_merged.bin        ← CAM merged    (flash @ 0x0, single-step)
│   ├── ESP32S3_CAM_AP.bin               ← CAM app only  (flash @ 0x10000)
│   ├── ESP32S3_CAM_AP.bootloader.bin    ← CAM bootloader (flash @ 0x0)
│   ├── ESP32S3_CAM_AP.partitions.bin    ← CAM partition table (flash @ 0x8000)
│   └── boot_app0.bin                    ← OTA boot selector (flash @ 0xe000)
│
└── docs/
    ├── report/
    │   └── ESP32_Gesture_RC_Car_Project_Report_Updated.docx
    └── diagrams/
        ├── Smart_Car_Diagrams.docx
        └── Car Receiver & FPV System Connection Diagram.pdf
```

Both binaries were compiled with the ESP32 Arduino core, 240 MHz, 4 MB flash (ESP32 DevKit V1, 30-pin).

---

## 1. Flash the ready-made binaries

The `.bin` files are complete images and go to **address 0x0**.

```
esptool.py --chip esp32 --port COM5 --baud 921600 write_flash -z 0x0 firmware/Transmitter_Glove.bin
esptool.py --chip esp32 --port COM6 --baud 921600 write_flash -z 0x0 firmware/Receiver_Car.bin
```

Or use an ESP32 flasher (e.g. "ESP32 Flasher"): pick the `.bin`, set offset `0x00000`, flash size `4MB`, baud `460800`.

> You can flash either unit over USB at any time — no MAC address or channel needs to be configured. The glove transmits to the broadcast address and the car answers back automatically.

---

## 2. Rebuilding from source

**Arduino IDE** — install these libraries via Library Manager:

* Adafruit SSD1306
* Adafruit GFX Library
* Adafruit BusIO (installed automatically as a dependency)

Board: *ESP32 DevKit V1*. The MPU6050 needs **no** library - it is driven by direct I2C register access, so there is no version mismatch to worry about. PSRAM is not required for the car firmware.

`WebPage.h` must be in the same folder as `Receiver_Car.ino`. It is separate on purpose: the Arduino `.ino` preprocessor scans the sketch for function signatures, and the JavaScript in the page (`async function poll()`) makes it inject invalid prototypes into the translation unit. Keeping the markup in a header keeps both the Arduino IDE and PlatformIO builds clean. The page lives in flash (PROGMEM), so it costs no RAM.

**PlatformIO** — project layout:

```
PRPWA/
├── platformio.ini
├── Receiver_Car/
│   ├── Receiver_Car.ino
│   └── WebPage.h
└── Transmitter_Glove/
    └── Transmitter_Glove.ino
```

Build targets: `receiver_car` and `transmitter_glove`.

---

## 3. Wiring

### Transmitter (glove)

| Signal | ESP32 pin | Notes |
|---|---|---|
| MPU6050 + OLED SDA | D21 (GPIO21) | shared I2C bus, 400 kHz |
| MPU6050 + OLED SCL | D22 (GPIO22) | |
| Mode button | D4 (GPIO4) | other leg to GND, internal pull-up |

MPU6050 at `0x68`, OLED at `0x3C`. Power both from **3V3**.

### Receiver (car)

| Signal | ESP32 pin | L298N / sensor |
|---|---|---|
| IN1 | D18 (GPIO18) | left side direction |
| IN2 | D19 (GPIO19) | left side direction |
| IN3 | D23 (GPIO23) | right side direction |
| IN4 | D14 (GPIO14) | right side direction |
| ENA (PWM) | D25 (GPIO25) | left side speed (LEDC ch0) |
| ENB (PWM) | D26 (GPIO26) | right side speed (LEDC ch1) |
| TRIG | D27 (GPIO27) | HC-SR04 Trig |
| ECHO | D32 (GPIO32) | HC-SR04 Echo (direct, sensor runs at 3.3 V) |

**Remove the ENA/ENB jumpers on the L298N** — the ESP8266 drives those pins directly.

**HC-SR04 runs at 3.3 V in this build.** Power its VCC from the ESP32 3V3 pin, so ECHO is 3.3 V safe and connects directly to D32 (no voltage divider). TRIG goes to D27. (Range may be slightly reduced compared with 5 V operation.)

**Power:** battery pack (7.4 V, series) into the L298N `12V`/`+` terminal, L298N `GND` to NodeMCU `GND`, and power the NodeMCU from the L298N 5 V output (or its own 5 V source). **All grounds must be common.** Never have USB and the battery connected at the same time.

**Charging is OFF-CAR only:** the series 7.4 V pack cannot be charged by a single 3.7 V TP4056. Remove both 18650s, wire them in parallel (3.7 V) and charge them with the TP4056 USB-C module, then reinstall. Never charge in-car or while the car is running.

**Boot note:** The mode button sits on GPIO4 (D4), a safe GPIO with no strapping/boot restrictions. The firmware enables `INPUT_PULLUP`, so no external resistor is needed.

If one side of the car drives backwards, swap that motor's two leads (or IN1/IN2).

---

## 4. Using it

### Mode button
Each press cycles **GESTURE → MOBILE WEB → FOLLOW ME → GESTURE**. The OLED updates instantly and the car switches within ~50 ms.

### Mode 1 — Gesture
Tilt the glove past 18° on an axis; it releases at 10° (hysteresis, so it does not flicker at the edge).
Forward/back = pitch, left/right = roll.

If an axis feels inverted on your build, flip these at the top of `Transmitter_Glove.ino`:

```cpp
#define INVERT_PITCH      false
#define INVERT_ROLL       false
```

The OLED shows the live pitch/roll and the direction being sent.

### Mode 2 — Mobile web
1. On the phone, join Wi-Fi **`Smart_Car_AP`** / password **`12345678`**.
2. Open **http://192.168.4.1**.
3. Hold a button to drive, release to stop. Arrow keys and space also work on a desktop.

The page only accepts commands while the car is actually in MOBILE WEB mode; otherwise it shows a banner telling you to press the glove button. While a button is held the page sends a heartbeat every 150 ms — if the phone disconnects, the car stops on its own after 700 ms.

### Mode 3 — Follow me
| Distance | Action |
|---|---|
| < 15 cm | reverse |
| 15 – 30 cm | hold position |
| 30 – 60 cm | drive forward |
| > 60 cm | stop (target lost) |

Thresholds use 3 cm hysteresis (start forward above 33 cm, keep reversing until 15 cm) so the car cannot chatter on a boundary. A lost echo for more than 400 ms is treated as "target lost". The measured distance is sent back to the glove and shown on its OLED.

---

## 5. Behaviour you should know about

* **No link in gesture mode = no movement.** If no valid packet arrives for 600 ms the car stops. Same for web mode with a stale heartbeat.
* **Every direction change ramps through zero.** The PWM slews at 110 counts per 20 ms, and the H-bridge is never reversed while the motors are still spinning — this protects the gearboxes and the battery.
* **Mode changes always stop the car first.**
* The OLED on the glove shows a filled dot + `LINK OK` only when the car's ACK packets are actually arriving, so it reflects the real link, not just "I sent something".
* **Obstacle safety (receiver):** in Gesture and Mobile Web modes, if something is closer than 15 cm in front, the Forward command is blocked automatically. Left, Right and Backward keep working. The dashboard shows an "Obstacle ahead" banner.

---

## 6. Tuning

| What | Where |
|---|---|
| Speed | `SPEED_STRAIGHT`, `SPEED_TURN`, `SPEED_FOLLOW` in `Receiver_Car.ino` |
| Acceleration ramp | `SLEW_STEP` |
| Gesture sensitivity | `TILT_ENTER_DEG`, `TILT_EXIT_DEG`, `TILT_FILTER_ALPHA` |
| Follow distances | `FOLLOW_NEAR_CM`, `FOLLOW_FAR_CM`, `FOLLOW_MAX_CM`, `FOLLOW_HYST_CM` |
| Wi-Fi name / password | `AP_SSID`, `AP_PASS` |
| Serial debug | `DEBUG_SERIAL 0` to disable logging |
