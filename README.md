# RePulse Firmware

![band](https://img.shields.io/badge/band-ESP32--C3-informational)
![bedside](https://img.shields.io/badge/bedside-ESP32--C3-informational)
![contract](https://img.shields.io/badge/BLE%20contract-v2.5-blue)

## Overview

Device-side code for RePulse, a sleep and heart monitoring system built for Indonesia Inventors Day 2026. The band measures the wrist and runs the escalation ladder on its own; the bedside measures the room, drives the actuators, and can sound its own siren when no phone is reachable. There is no display on either device — the phone is the only screen. Every byte on the air follows `../BLE_GATT_CONTRACT.md` v2.5 — the Android app was built against that document first, so the contract wins any disagreement.

## Tech Stack

- **Arduino framework** on the ESP32 core 3.x (arduino-cli 1.5+)
- **NimBLE-Arduino 2.x** — BLE peripheral and observer, chosen over Bluedroid for flash footprint
- **ArduinoJson 7.x** — the JSON payloads the contract defines (`0005`, `0009`, bedside `0003`)
- **SparkFun MAX3010x** (FIFO access only — the beat and SpO2 maths are ours, in `ppg.h`) and **Adafruit MPU6050** — band sensors
- **BH1750**, **Adafruit DHT**, **Adafruit NeoPixel**, **DFRobotDFPlayerMini**, **ESP_I2S** — bedside sensors and actuators

## Features

**Band**

- Full GATT surface: vitals, SpO₂ and body position, motion, SOS, config, offline buffer, escalation, battery and clock, commands. `000A` (ECG) is implemented but disabled — no AD8232 in the final circuit
- Escalation ladder runs entirely in firmware, phone present or not — stages, timings, and cancel-on-movement all match contract §3.5
- Escalation stage broadcast in the advertising packet (§2.1), with `phone_connected` meaning a *live* app — connected and heard from in the last 30 s — because Android keeps the link open after it has stopped the app's JavaScript
- `stand_down` command (§3.8), the only way off stage 4
- Offline ring buffer, 256 entries, erased only after the app acknowledges the flush
- Vibration motor gated against the accelerometer so the alarm cannot cancel itself
- Worn detection and a 0–15 signal quality figure, the two flags that keep a band on a table from raising an alert
- Non-blocking SpO₂ sampling — the old Firebase sketch blocked for four seconds per reading, which the ladder cannot afford

**Bedside**

- Room sensing on the contract's own units: °C×10, %RH×10, lux×100, whole dB
- Snore detection by repeating 0.2–0.5 Hz pattern, not by average decibels — a fan is loud but flat, and the contract rejects the flat measure explicitly
- Exponential dimming with blue suppressed during sunset, because linear PWM reads as a sudden blackout at the tail. Sunset starts at the commanded brightness and always ends dark; when the app sends `light.rgb` (the user's chosen colour) it is drawn exactly, blue included
- Aroma safety limits enforced in firmware: 30 s per event, 4 events per night, over-limit requests **refused** rather than silently shortened. The limit is checked before the hardware is, so an over-long request is refused whether or not a diffuser is connected
- Autonomous siren under contract §2.1 — sounds only when the band broadcasts stage ≥ 3, the MAC matches the paired band, the bedside has no live app, and the band reports no live phone either — live meaning connected and heard from in the last 30 s
- Paired band MAC stored in NVS, so it survives a power cut

## Prerequisites

- [arduino-cli](https://arduino.github.io/arduino-cli/) 1.5 or newer, or the Arduino IDE 2.x
- ESP32 board package 3.x by Espressif
- A C++17 compiler (`g++`, `clang++`, or MSVC) to run the host tests — only used off-device

## Installation

1. Install the board package:

   ```bash
   arduino-cli core install esp32:esp32
   ```

2. Install the libraries:

   ```bash
   arduino-cli lib install "NimBLE-Arduino" "ArduinoJson"
   arduino-cli lib install "SparkFun MAX3010x Pulse and Proximity Sensor Library" "Adafruit MPU6050"
   arduino-cli lib install "BH1750" "DHT sensor library" "Adafruit NeoPixel" "DFRobotDFPlayerMini"
   ```

3. Confirm the pin block at the top of each sketch against the real wiring, then build:

   ```bash
   arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app" repulse_band
   arduino-cli compile --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app" repulse_bedside
   ```

   `CDCOnBoot=cdc` is not optional. The C3 defaults to routing `Serial` to GPIO 21/20 instead of USB, and the serial monitor stays blank while the firmware runs perfectly.

   **An empty serial monitor has three causes, in the order worth checking.**
   First, the flag above — wrong routing prints nothing, ever. Second,
   enumeration: a native-USB port only appears one to two seconds after
   reset, so anything printed before that is gone. `setup()` now waits for
   the host, capped at two seconds so a cable-free band still boots. Third,
   silence is not death — the band used to print nothing at all once
   `setup()` finished, so a healthy device and a hung one looked identical.
   `SERIAL_HEARTBEAT` now prints one labelled `[GELANG]` line per second.

   If the monitor still stays blank, close it, press reset on the board, and
   reopen it. The USB peripheral re-initialises when the sketch starts, and
   most serial monitors do not follow the port back.

4. Flash, one board at a time:

   ```bash
   arduino-cli upload --fqbn "esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app" -p COM5 repulse_band
   ```

## Configuration

Firmware has no `.env`; these are the compile-time constants that must match your hardware.

**Band — `repulse_band/repulse_band.ino`**

| Constant | Description | Example | Required |
|----------|-------------|---------|----------|
| `PIN_I2C_SDA` / `PIN_I2C_SCL` | Shared I²C bus, MAX30102 (`0x57`) and MPU6050 (`0x68`) | `10` / `0` | Yes |
| `PIN_BUTTON` | SOS button, active LOW with internal pull-up | `3` | Yes |
| `PIN_MOTOR` | Vibration motor, PWM via LEDC | `1` | Yes |
| `PIN_ECG_OUT` | AD8232 analog output, must be ADC1. `-1` disables the ECG characteristic | `-1` | No — not fitted |
| `PIN_ECG_LO_P` / `PIN_ECG_LO_N` | AD8232 lead-off detect, drives `lead_on` | `-1` / `-1` | No — not fitted |
| `PIN_BATTERY_ADC` | Battery divider. `-1` reports `255` — contract §3.6 for "not measured" — instead of inventing a number | `-1` | No — not fitted |
| `PPG_IR_PRESENT` | Infrared DC level separating "on a wrist" from "on a table". Lives in `ppg.h`; override it with a `#define` before the include | `30000` | Comes with the LED current — it assumes `0xFF`. `[CAL]` prints `IR=` twice a second; read it on the wrist **and** on a table, then set this between the two |

**Bedside — `repulse_bedside/repulse_bedside.ino`**

| Constant | Description | Example | Required |
|----------|-------------|---------|----------|
| `PIN_MIC_BCLK` / `PIN_MIC_WS` / `PIN_MIC_DIN` | INMP441 I²S microphone | `5` / `4` / `6` | Yes |
| `PIN_I2C_SCL` / `PIN_I2C_SDA` | BH1750. SDA stays off GPIO9 — see hardware notes | `7` / `1` | Yes |
| `PIN_LED` | WS2812 data in | `8` | Yes |
| `PIN_DHT` | DHT11 data, 10 kΩ pull-up to 3V3. Change the `DHT` constructor for a DHT22 | `10` | Yes |
| `PIN_DFPLAYER_RX` / `PIN_DFPLAYER_TX` | DFPlayer Mini, 1 kΩ in series on TX | `2` / `3` | Yes |
| `PIN_AROMA` | Diffuser via MOSFET or relay. `-1` = no control wired, aroma commands answer `status = 1` | `20` | Yes — relay fitted |
| `AROMA_ACTIVE_LEVEL` | Measured, not assumed. Most relay modules are active LOW; the one on this bench is active HIGH. Wrong polarity runs the diffuser all night | `HIGH` | Yes |
| `LED_COUNT` | Number of WS2812 pixels on the strip | `60` | Yes |
| `LED_MAX_BRIGHTNESS` | Ceiling, so a 2 A supply is not asked for full white | `100` | Yes |
| `MIC_DB_OFFSET` | dB calibration. Every snore threshold is meaningless until this matches a real room | `26.0` | Yes |
| `TRACK_SIREN` | microSD track the siren plays at volume 30 | `4` | Yes |

### Hardware notes

- **ESP32-C3 strapping pins are GPIO2, GPIO8, and GPIO9.** GPIO9 low at reset puts the chip in download mode: no firmware runs, the serial monitor stays empty, and the onboard blue LED stays off. BH1750 SDA used to sit on GPIO9 on the theory that the module's pull-up holds it high — until a loose or failing module pulled it low and the board went silent. It now lives on GPIO1, and GPIO9 stays unused. To confirm this failure on a silent board, close the serial monitor and run `esptool --chip esp32c3 -p COMx --before no-reset --after no-reset read-mem 0x60004038`: bit 3 is GPIO9, bit 2 is GPIO8, and `0x5` means download mode where a normal boot reads `0xc`. GPIO8 (WS2812 data) and GPIO2 (DFPlayer TX) are still strapping pins and float at boot — fit a 10 kΩ pull-up to 3V3 on each so the board does not boot differently depending on which module powers up first.
- **WS2812 at 5 V fed 3.3 V data** is marginal. Either power the strip from 4.5 V, add a 74AHCT125 level shifter, or accept that it may work on your strip and not the next one.
- **A 5 V 2 A supply will not drive 60 pixels at full white** — that is roughly 3.6 A. `LED_MAX_BRIGHTNESS` holds it near 1.4 A. Measure before raising it, or fit a 5 V 5 A supply.
- **The diffuser relay is on GPIO20 and its off level is written before `pinMode()`.** A floating output reads LOW, and an active-LOW relay module would then run the diffuser from every boot — the exact condition §4.3 warns about. If a replacement module switches on at boot and off on command, invert `AROMA_ACTIVE_LEVEL` and change nothing else.
- The band's I²C, button, and motor pins came from the working `monitoring_esp32_firebase` sketch and are the only pins verified against real hardware. Everything else is from a diagram.

## Usage

The band prints a boot summary and then reports over USB serial:

```
=== RePulse Band (ESP32-C3) ===
[HR]  MAX30102... OK
[IMU] MPU6050... OK
[BLE] Advertising as RePulse Band
[BLE] Connected: 7c:2a:31:04:9f:e1
[LADDER] stage=1 reason=2
```

Then one labelled line a second, which is how you tell a working band from
a hung one without a debugger:

```
[GELANG] BPM=68(sah)  RR=880 ms(sah)  SpO2=97%(sah)  gerak=42 mG  posisi=telentang  dipakai=YA  sinyal=13/15  tahap_alarm=0  alasan=0  baterai=n/a  HP=tersambung
```

`baterai=n/a` is correct until a divider is fitted — contract §3.6 reports
`255` for "not measured" rather than inventing a percentage.

The bedside reports its room readings and every safety decision:

```
=== RePulse Bedside (ESP32-C3) ===
[BH1750] OK
[DFPLAYER] OK
[INMP441] OK
[BLE] Advertising as RePulse Bedside, memindai gelang
[SCAN] Gelang dipasangkan: 84:f7:03:11:2c:9a
[ROOM] 29.1°C 74.0% 0.40 lux 38 dB
[SNORE] terdeteksi intensitas=42
[AROMA] DITOLAK — batas keselamatan
```

Run the host tests before trusting a change to any of the pure logic:

```bash
cd repulse_band/test    && g++ -std=c++17 -o ladder_test  ladder_test.cpp  && ./ladder_test
cd repulse_band/test    && g++ -std=c++17 -o ppg_test     ppg_test.cpp     && ./ppg_test
cd repulse_bedside/test && g++ -std=c++17 -o bedside_test bedside_test.cpp && ./bedside_test
```

## BLE Surface

Both devices advertise their service UUID. The app filters on the UUID, never the name.

**Band** — `RePulse Band`, service `4FA10000-8C3A-4B8F-A292-3E83D02F1A00`

| Characteristic | Data | Type | Rate |
|---|---|---|---|
| `0001` | Status byte + up to 5 × (bpm, rr_ms) | Notify | 1 s |
| `0002` | SpO₂ percent, body position | Notify | 20 s, configurable |
| `0003` | Motion level in milli-g | Notify | 1 s |
| `0004` | SOS button held ≥ 2 s | Indicate | on press |
| `0005` | Config JSON, app → band | Write | as needed |
| `0006` | Offline buffer flush | Notify + Write | on reconnect |
| `0007` | Escalation stage and reason | Indicate | on change |
| `0008` | Battery percent, charging, epoch | Read / Notify | 5 min |
| `0009` | Command JSON, app → band | Write | as needed |
| `000A` | ECG samples, 90 per packet | Notify | never — no AD8232 fitted |

The band's advertising packet also carries manufacturer data `FF FF 01 <stage> <flags>`, flag bit 7 `worn` and bit 0 `phone_connected`. That is what the bedside listens for.

**Bedside** — `RePulse Bedside`, service `4FA20000-8C3A-4B8F-A292-3E83D02F1A00`

| Characteristic | Data | Type | Rate |
|---|---|---|---|
| `0001` | temp ×10, RH ×10, lux ×100, dB | Notify | 1 min |
| `0002` | Snore flag and intensity | Notify | on change |
| `0003` | Actuator JSON, app → bedside | Write | as needed |
| `0004` | Command id and status | Indicate | on completion |

## Answers to contract §7

| # | Question | Answer |
|---|---|---|
| 1 | Any characteristic you cannot deliver? | None. All ten band and four bedside characteristics are implemented; band `000A` stays silent because the final circuit has no AD8232 |
| 2 | Can both devices negotiate MTU 185? | Both request it. Not yet measured against a real phone |
| 3 | Offline ring buffer capacity? | 256 entries, 2.3 KB of RAM |
| 4 | Does the band clock survive a restart? | No. The C3 has no RTC, so `sync_time` is required on every connect |
| 5 | BH1750 fitted on the bedside? | Yes, I²C on GPIO 7 (SCL) and GPIO 1 (SDA) — SDA moved off strapping pin GPIO9, see hardware notes |
| 6 | Worn/removed detection method? | Infrared DC level against `PPG_IR_PRESENT` in `ppg.h`, with a 250 ms debounce so a wobbling contact does not throw away the settle progress. LEDs run at full current (`0xFF`) with the ADC range at 16384 nA for headroom. Still needs measuring on the wrist that will wear it |
| 7 | How is signal quality 0–15 derived? | Linear map of infrared headroom above the worn threshold, capped at 4 when no beat has been seen for 4 seconds. Calibration should gate at 8 or above |
| 7a | Why did SpO₂ read a constant 16%? | The Maxim algorithm raised `spo2Valid` on noise. It is gone. `ppg.h` computes saturation from a one-second AC-RMS window and discards any window whose R or perfusion index falls outside the range AN6409 is valid over — the two figures that tell a pulse from a movement artefact |
| 7b | Why was BPM stuck at 0 with a perfect IR level? | SparkFun's `checkForBeat` is 16-bit inside: `averageDCEstimator` returns `int16_t`, so any DC above 32767 comes back negative and the AC signal is zero forever. It was worked around with an invented midpoint and a clamp. Both are gone — `ppg.h` does the detection itself in floating point, with a two-pole band-pass that removes breathing (which swings larger than the pulse at the wrist) and a threshold that follows the peak envelope. Worst-case BPM error against a synthetic waveform is 3.8% from 50 to 170 bpm |
| 8 | Accelerometer gating: time or amplitude? | Time. Motion is ignored while the motor runs plus 200 ms — an amplitude threshold has to be re-measured every time the motor or strap changes |
| 9 | Does the stage broadcast fit the main advertising packet? | Yes, 28 of 31 bytes. The device name moved to the scan response |
| 10 | Can the bedside scan and store the band MAC? | Yes, first band seen is paired and the MAC is written to NVS |
| 11 | Is the AD8232 fitted? Sample rate and ADC resolution? | No. It was dropped from the design — pulse comes from the MAX30102 alone. The 250 Hz / 12-bit code stays behind `PIN_ECG_OUT >= 0` for whoever fits one |
| 12 | Is lead-off wired and reportable? | Moot. No electrodes to come off |
| 13 | Does ECG recording disturb the PPG reading? | Moot. There is no ECG recording |
| 14 | When will firmware be ready to test? | Both sketches compile. Testing needs wired devices |
| 15 | Objections to the document? | Two gaps in the wiring, not objections to the document. The bedside has no siren output, so the siren plays through the DFPlayer at volume 30 with the lamp flashing white — white noise and siren therefore cannot sound together, which matches §PRD 6.3 where an emergency switches white noise off anyway. The bedside aroma relay is now fitted on GPIO20 |

## Project Structure

```
firmware/
├── README.md
├── repulse_band/               # ESP32-C3 band, the wrist
│   ├── repulse_band.ino        # pins, sensors, anomaly detection, main loop
│   ├── ble_band.h/.cpp         # GATT surface and the §2.1 advertising payload
│   ├── ppg.h                   # optical DSP: contact, beat, SpO2 — no Arduino headers
│   ├── ladder.h                # escalation state machine — no Arduino headers
│   ├── evtbuf.h                # offline ring buffer, erase-after-ACK
│   └── test/                   # host asserts for the three headers above
└── repulse_bedside/            # ESP32-C3 bedside, the room
    ├── repulse_bedside.ino     # pins, sensors, actuators, main loop
    ├── ble_bedside.h/.cpp      # GATT §4 plus the band advertising scanner
    ├── snore.h                 # 0.2-0.5 Hz pattern detection — pure
    ├── aroma.h                 # safety limiter, refuses rather than clamps
    ├── light.h                 # exponential ramp, kelvin to RGB, blue suppression
    └── test/bedside_test.cpp   # host asserts for the three headers above
```

Wire-by-wire build instructions for both boards are in `WIRING.md`.

## Contributing

Fork, branch, pull request. Three rules specific to this folder:

- Byte layouts come from `../BLE_GATT_CONTRACT.md`. If a payload has to change shape, change the contract first — the Android app decodes against it in `repulse-mobile-app/src/ble/codec.ts` and will break silently otherwise.
- The `.h` files listed above stay free of Arduino headers so they can run on a laptop. Anything that touches them needs its assert in the matching `test/` file.
- Safety limits are enforced in firmware and in the app independently, on purpose. Do not remove the firmware side because the app already checks — one of them will be wrong eventually.

Deliberate shortcuts are marked with a `ponytail:` comment naming the ceiling and the upgrade path. Read those before assuming something is an oversight.

## License

No license file exists in this repository yet. Treat the code as internal to the RePulse project until one is added.
