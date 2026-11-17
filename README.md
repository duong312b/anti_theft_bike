# Anti-Theft Motorbike Tracker (ESP32 + ESP-IDF)

An ESP32 device that hides in a motorbike. When the bike is armed and the MPU6050 detects abnormal movement (pushed, lifted, tilted or ridden), the device reads the GPS position from the A60C module and sends an SMS with a Google Maps link to a configured phone number. It keeps sending tracking SMS for a while after that. A 16x2 I2C LCD shows the coordinates and the system status.

```
            +-----------+     I2C      +-----------+
            |  MPU6050  |<------------>|           |
            +-----------+  INT ------> |           |      UART1 (AT)      +----------------+
                                       |   ESP32   |<-------------------->|  A60C GSM/GPS  |----> SMS to phone
            +-----------+     I2C      |           |      UART2 (NMEA)    |  + SIM card    |
            | LCD 16x2  |<------------>|           |<---------------------|                |
            | (PCF8574) |              +-----------+  RST/DTR/RI -------->+----------------+
            +-----------+
```

## Features

- **Two-stage motion detection**:
  1. The MPU6050 hardware motion interrupt wakes the ESP32 on any bump.
  2. The ESP32 then samples at 100 Hz for 2 s. It confirms theft only if the movement continues, or if the bike's tilt changes by more than 15° from its pose when armed. Single knocks, wind and people leaning on the bike are ignored.
- **SMS alert** with a clickable `https://maps.google.com/?q=lat,lon` link, speed, satellite count, and whether the fix is live or the last known one.
- **Tracking mode**: after the first alert, sends N more SMS every T seconds, then re-arms.
- **LCD** switches between a position page (lat/lon) and a status page (state, satellites, UTC time).
- Pins, phone number, thresholds and timings are all set in `idf.py menuconfig`.

## Hardware

| Part | Notes |
|---|---|
| ESP32 DevKit (ESP32-WROOM-32) | Any classic ESP32 board |
| MPU6050 (GY-521) | Accelerometer + gyroscope, I2C address 0x68 |
| A60C GSM + GNSS module | Needs a 2G/4G SIM that can send SMS. AT commands on TXD/RXD, NMEA on GNSS_TX |
| 16x2 LCD with I2C backpack (PCF8574) | Address 0x27 (or 0x3F) |
| Power | Bike 12 V → buck converter. The GSM module draws **up to 2 A peaks** while transmitting |

## Wiring (default GPIOs, all changeable in menuconfig)

### MPU6050

| MPU6050 pin | Connect to | Notes |
|---|---|---|
| VCC | 3V3 | GY-521 boards also accept 5 V |
| GND | GND | |
| SCL | **GPIO22** | Shared I2C bus |
| SDA | **GPIO21** | Shared I2C bus |
| XCL | not connected | Auxiliary I2C master, unused |
| XDA | not connected | (if your board has it) |
| AD0 | GND | Address 0x68 (tie to 3V3 → 0x69, and change `MPU6050_ADDR_AD0_LOW` in `motion.c`) |
| INT | **GPIO27** | Motion interrupt, active high |

### A60C GSM/GNSS module

| Module pin | Connect to | Notes |
|---|---|---|
| VIN | 5 V / 4 V supply (see module datasheet) | **Separate supply, ≥ 2 A.** Add a 470–1000 µF capacitor next to the module |
| GND | GND | Common ground with the ESP32 |
| TXD | **GPIO16** (ESP32 RX, UART1) | AT command responses |
| RXD | **GPIO17** (ESP32 TX, UART1) | AT commands |
| GNSS_TX | **GPIO26** (ESP32 RX, UART2) | NMEA sentences ($GxRMC, $GxGGA) |
| GNSS_RX | **GPIO25** (ESP32 TX, UART2) | Optional (GNSS configuration) |
| RST | **GPIO33** | Hardware reset (active low by default) |
| DTR | **GPIO32** | Held low = module stays awake (for future sleep support) |
| RI | **GPIO35** | Ring indicator on incoming SMS (input only; reserved for SMS commands) |

> ⚠️ **Logic levels**: check that the module's UART is 3.3 V. If it is 2.8 V or 1.8 V, or it runs at 5 V, use a level shifter.

### LCD 16x2 (I2C)

| LCD pin | Connect to |
|---|---|
| VCC | 5 V (most backpacks need 5 V for contrast) |
| GND | GND |
| SDA | **GPIO21** |
| SCL | **GPIO22** |

> When the LCD runs at 5 V, its backpack pull-ups pull SDA/SCL to 5 V. Remove those pull-ups, or use a level shifter, and put 4.7 kΩ pull-ups to 3.3 V on SDA/SCL.

### GPIO summary

| GPIO | Function |
|---|---|
| 21 / 22 | I2C SDA / SCL (MPU6050 + LCD) |
| 27 | MPU6050 INT |
| 17 / 16 | UART1 TX / RX → modem AT |
| 25 / 26 | UART2 TX / RX → GNSS NMEA |
| 33 | Modem RST |
| 32 | Modem DTR |
| 35 | Modem RI |
| 1 / 3 | UART0 (USB console, don't use) |

Avoid strapping pins (0, 2, 5, 12, 15) and flash pins (6–11) when you change these.

## Project structure

```
anti_theft_bike/
├── CMakeLists.txt
├── sdkconfig.defaults
├── README.md
├── main/
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild     # menuconfig: pins, phone number, thresholds
│   ├── app.h                 # shared types, states, ports
│   ├── main.c                # init + alarm state machine + SMS formatting
│   ├── motion.c              # MPU6050 INT + theft confirmation logic
│   └── display.c             # LCD pages
└── components/
    ├── mpu6050/              # MPU6050 driver (i2c_master API)
    ├── lcd_i2c/              # HD44780 over PCF8574 driver
    ├── gnss/                 # NMEA parser task (RMC, GGA)
    └── modem/                # AT command layer, SMS sending
```

### Software architecture

| Task | Priority | Role |
|---|---|---|
| `motion` | 6 | Wakes on MPU INT (or every 1 s), confirms theft, sets `APP_EVT_THEFT` |
| `gnss` | 5 | Reads UART2, parses NMEA, keeps the latest fix behind a mutex |
| `main` (app_main) | 1 | State machine, sends SMS through the modem |
| `display` | 3 | Updates the LCD every second |

State machine:

```
 DISARMED ──arm──> ARMING ──(ARM_DELAY)──> ARMED ──theft──> ALARM
                     ^                                        │
                     └────── after TRACK_MAX_SMS tracking SMS ┘
```

## Build and flash

Requires ESP-IDF v5.2 or newer (uses the new `i2c_master` driver). Tested against v6.x.

```bash
. $HOME/esp-idf/esp-idf/export.sh
cd anti_theft_bike
idf.py set-target esp32
idf.py menuconfig          # → "Anti-Theft Bike Configuration"
idf.py -p /dev/ttyUSB0 flash monitor
```

In **menuconfig → Anti-Theft Bike Configuration**, set at least:

- **Modem / SMS → Phone number**: e.g. `+84912345678`
- **Modem / SMS → GNSS enable command**: the default is `AT+GPS=1` (Ai-Thinker A9G style). Check the A60C AT manual and change it if needed.
- **Pin assignment**: if your wiring differs from the table above.

## Example SMS

```
ALERT! Bike moved.
https://maps.google.com/?q=21.028511,105.804817
Spd:0km/h Sat:8 LIVE
```

## Tuning motion detection

| Option | Default | Effect |
|---|---|---|
| `ATB_MPU_MOT_THR` | 20 (~40 mg) | Hardware wake sensitivity. Lower = wakes more often |
| `ATB_ACCEL_THR_MG` | 150 mg | Accel deviation from 1 g that counts as "active" |
| `ATB_GYRO_THR_DPS` | 20 °/s | Rotation rate that counts as "active" |
| `ATB_CONFIRM_WINDOW_MS` | 2000 | How long to watch after a wake-up |
| `ATB_CONFIRM_MIN_ACTIVE` | 40 | Active samples (of 200) needed to trigger |
| `ATB_TILT_THR_DEG` | 15° | Tilt change from the armed pose that triggers on its own |

To tune, run `idf.py monitor` and watch the `motion: confirm: active=X/200 tilt=Y` log lines while you shake, push and lean on the bike.

## Troubleshooting

| Symptom | Check |
|---|---|
| `no answer to AT` | TX/RX swapped? Baud rate (try 9600/115200)? Module powered with ≥ 2 A? |
| `SIM not ready` | SIM inserted the right way? PIN lock disabled? |
| Brownout / reboot when SMS is sent | Power supply is too weak. Add a bulk capacitor and use a separate regulator |
| `GPS searching...` forever | GNSS antenna connected and has a view of the sky? GNSS enable command right? Cold start can take 1–5 min |
| `WHO_AM_I = 0x70` warning | MPU6050 clone, usually fine |
| LCD blank / boxes | Adjust the contrast potentiometer. Try address 0x3F |

## Roadmap / TODO

- [ ] Arm/disarm by SMS command (`ARM`, `DISARM`, `STATUS`) from the owner's number (use the RI pin + `AT+CMGR`)
- [ ] Physical arm switch, or BLE/RFID key fob
- [ ] Siren / buzzer output
- [ ] Save settings in NVS (phone number, armed state across reboot)
- [ ] Deep sleep with the MPU INT as the wake source, to save the bike battery
- [ ] Monitor the bike battery voltage (ADC) and send a "power cut" alert
- [ ] Retry `modem_init()` when it fails, and watch network registration
- [ ] Send the position over GPRS/MQTT as well as SMS
