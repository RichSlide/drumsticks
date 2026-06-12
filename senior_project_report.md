# Wireless IMU-Based Drumstick Controllers

**A Senior Project Report**

presented to

the Faculty of California Polytechnic State University

San Luis Obispo

In Partial Fulfillment of the Requirements for the Degree

**Bachelor of Science in Computer Engineering**

By

Tyler Bodenhamer

June 2026

---

## Table of Contents

1. Abstract
2. Introduction
   - 2.1 Motivation
   - 2.2 Project Goals and Objectives
   - 2.3 Project Deliverables
   - 2.4 Project Outcomes
3. Background
   - 3.1 Existing Products
   - 3.2 Relevant Technologies
4. Formal Project Definition
   - 4.1 Customer Requirements
   - 4.2 Engineering Requirements Table
   - 4.3 User Persona
   - 4.4 Use Cases
5. System Design
   - 5.1 System-Level Overview
   - 5.2 Hardware Design
   - 5.3 Firmware Design
   - 5.4 Python Host Software Design
6. Problems Faced and Solutions
7. System Testing and Analysis
8. Conclusion and Future Work
9. Reflection
10. Bibliography
11. Appendix A — Bill of Materials
12. Appendix B — Source Code
13. Appendix C — Senior Project Analysis Form

---

## 1. Abstract

This project presents a pair of wireless electronic drumstick controllers built on the ESP32-C3 SuperMini microcontroller. Each stick contains an ICM-20948 nine-degree-of-freedom inertial measurement unit (IMU) and communicates over Bluetooth Low Energy (BLE) to a Python host application running on a Windows PC. The firmware runs a complementary filter and a custom hit-detection pipeline at 100 Hz, classifying each downstroke by wrist rotation angle and transmitting the result to the host. The host plays sampled drum and cymbal sounds through pygame with velocity-sensitive volume. A key challenge was achieving simultaneous audio playback from both sticks without perceptible delay; this was solved by characterizing the structural BLE scheduling offset and implementing a 9-millisecond software synchronization window, backed by the Windows multimedia timer API for 1 ms precision. Additional latency optimizations — widening the ICM-20948 digital low-pass filter bandwidth, increasing the FreeRTOS scheduler tick rate from 100 Hz to 1000 Hz, and requesting a minimum 7.5 ms BLE connection interval — reduced end-to-end impact-to-audio latency to 18–32 ms in final testing. The completed system supports three percussion zones (snare, rack tom, floor tom), a cymbal overlay mode triggered by a wrist button, and a real-time terminal visualization of stick position and recent hits that dynamically scales to the terminal window size. Total bill-of-materials cost for both sticks is under $25.

---

## 2. Introduction

### 2.1 Motivation

Electronic drum kits have existed for decades, but they universally require a physical pad for the drummer to strike. The pad senses impact through a piezoelectric sensor and sends a MIDI trigger to a sound module. This architecture imposes a significant physical footprint: even a minimal e-kit occupies several square feet of floor space and requires a rack, cymbal arms, and a hi-hat stand. Practicing in a small apartment, a dormitory room, or a hotel room is effectively impossible.

The alternative — air drumming with no feedback — exists as a novelty in several consumer products, but these products have historically suffered from poor hit detection and no meaningful zone discrimination. A drummer cannot reliably play a snare pattern on the left stick and a ride cymbal pattern on the right stick because the software cannot tell where in space each stick was when it struck an imaginary surface.

I play drums and wanted a practice tool I could use anywhere with no setup other than a laptop. The core insight driving this project is that the key information a drummer needs is already present in the motion of the stick: the magnitude of the impact (peak acceleration), the direction of the downswing (pitch gyro rate), and the rotational heading of the wrist at the moment of impact (yaw integration). If these three signals can be captured reliably from an IMU at high enough sample rate and transmitted to a PC with low enough latency, a realistic wireless air drum kit becomes achievable.

### 2.2 Project Goals and Objectives

**Goals:**

- Build two wireless drumstick controllers that can be worn or held naturally while air drumming.
- Detect hits reliably without false positives or missed notes across a range of playing intensities.
- Classify each hit into one of at least three drum zones based on wrist heading.
- Play the correct sampled drum sound within a perceptually acceptable latency after each hit.
- Support simultaneous hits from both sticks without either sound being delayed or dropped.

**Objectives (measurable):**

- Hit detection latency from physical impact to audio output shall be less than 50 ms.
- False positive rate shall be less than 1 per 30 seconds of normal playing motion.
- Both sticks' sounds shall play within 15 ms of each other when struck simultaneously.
- Zone classification shall be stable; heading drift shall be less than 5 degrees per minute of stationary holding.
- System shall operate continuously for at least 30 minutes without reconnection or audio failure.

### 2.3 Project Deliverables

- Two functioning drumstick controller units, each containing an ESP32-C3 SuperMini and an ICM-20948 IMU.
- Firmware source code in C (ESP-IDF framework) for each stick.
- Python host application with BLE communication, real-time audio, and terminal visualization.
- This written project report documenting design decisions, problems encountered, and solutions.

### 2.4 Project Outcomes

The completed system allows a drummer to practice anywhere with no physical drum kit. Holding two sticks, the player can perform snare patterns, tom fills, and cymbal crashes in open air. The host laptop plays back realistic sampled sounds in real time. A button on each stick allows the player to recalibrate heading at any moment (short press) or switch to cymbal sounds (hold). A future musician or engineer who wants to continue this work has full source code, a documented BLE protocol, and a detailed account of every non-obvious problem that was encountered during development.

---

## 3. Background

### 3.1 Existing Products

**Aerodrums** (2014) uses a camera-based optical tracking system with retroreflective balls attached to sticks and feet. It achieves excellent accuracy but requires a camera stand, careful room lighting, and the retroreflective markers. It is not portable in any practical sense and retails for approximately $130.

**Freedrum** is a commercial IMU-based air drumming product that attaches small sensors to sticks and shoes. User reviews consistently report poor zone discrimination and frequent false positives; the product appears to have limited commercial success. Their approach uses proprietary firmware with no published technical details.

**General MIDI air drum apps** on smartphones use the phone's built-in accelerometer and are considered novelty items. Sample rates are too low (typically 50 Hz or less) and latency through the OS audio stack is unpredictable.

This project differs from all three in that the full firmware and algorithm source code is open and documented, making it a useful starting point for further development. It also specifically addresses the two-stick timing problem — a problem that is structurally inherent to BLE but which does not appear to be documented elsewhere.

### 3.2 Relevant Technologies

**ESP32-C3:** A RISC-V single-core microcontroller from Espressif with integrated 2.4 GHz BLE 5.0 and Wi-Fi. The SuperMini form factor is approximately 22 × 18 mm — small enough to embed in a stick grip. It runs FreeRTOS and is programmed with the ESP-IDF framework.

**ICM-20948:** A 9-DOF IMU (3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer) from TDK InvenSense. The device communicates over I2C or SPI. For this project the accelerometer is configured at ±2g (16-bit, 16384 LSB/g) and the gyroscope at ±250 dps (16-bit, 131 LSB/dps). The magnetometer is unused.

**Complementary filter:** A computationally lightweight sensor fusion technique that combines gyroscope integration (good short-term, drifts long-term) with accelerometer tilt estimation (good long-term, noisy during motion) using a weighted sum. For a sample period of 10 ms and a weight α = 0.95, it provides stable pitch estimation with minimal latency.

**BLE GATT Notifications:** BLE's server-initiated data push mechanism. The ESP32 acts as a GATT server and calls `ble_gatts_chr_updated()` each time it has a new IMU packet. The Python host subscribes using the `bleak` library. Notification delivery is bound by the BLE connection interval — data cannot arrive faster than once per interval.

**Python asyncio:** Python's cooperative multitasking framework. BLE callbacks from `bleak` are executed in the asyncio event loop. Any blocking call inside the loop starves all other callbacks — a critical constraint that drove a key architectural decision described in Section 6.

---

## 4. Formal Project Definition

### 4.1 Customer Requirements

As the sole designer and user of this system, I defined the requirements from my own experience as a drummer and from practical constraints of the intended use case (practice anywhere, minimal setup).

| # | Customer Requirement |
|---|----------------------|
| CR-1 | The system shall require no physical drum surfaces or pads |
| CR-2 | The system shall detect a drumstick downstroke and play the corresponding sound |
| CR-3 | The system shall support at least three distinct drum zones (e.g., snare, tom, floor tom) |
| CR-4 | The system shall support cymbal sounds as an alternate mode |
| CR-5 | The sound shall play within a perceptually acceptable delay after the physical impact |
| CR-6 | Both sticks shall be wireless and connect automatically |
| CR-7 | The system shall not produce false hits during normal arm movement |
| CR-8 | A button on the stick shall allow the player to recalibrate heading at any time |
| CR-9 | The system shall allow playing from both sticks simultaneously without one being delayed |
| CR-10 | The total hardware cost shall be affordable for a student |

### 4.2 Engineering Requirements Table

**Table 1: Engineering Requirements**

| Spec # | Parameter Description | Requirement / Target | Tolerance | Risk | Compliance |
|--------|-----------------------|---------------------|-----------|------|------------|
| ER-1 | Hit detection latency (impact to audio) | 50 ms | Max | H | T |
| ER-2 | IMU sample rate | 100 Hz | Min | L | T, I |
| ER-3 | Accelerometer threshold for hit | 2.0 g | Min | M | T |
| ER-4 | Downswing gate angular rate | 30 deg/s (gx) | Min | M | T |
| ER-5 | Hit cooldown (debounce) | 200 ms | Max | L | A, T |
| ER-6 | Heading drift (stationary) | 5 deg/min | Max | H | T |
| ER-7 | Two-stick simultaneity window | 15 ms | Max | H | T |
| ER-8 | False positive rate | 1 per 30 sec | Max | M | T |
| ER-9 | BLE connection interval | 15 ms | Max | M | A, I |
| ER-10 | Audio channels available | 32 | Min | L | I |
| ER-11 | Hardware cost per stick | $15 USD | Max | L | I |
| ER-12 | Continuous operation time | 30 min | Min | L | T |

**High-risk requirements discussion:**

*ER-1 (Hit latency):* This was the most user-perceptible requirement. Latency has several components: IMU sample period (10 ms), BLE notification slot (7.5–15 ms), Python callback scheduling (< 1 ms asyncio overhead), and pygame audio mixer buffer (256 samples at 44100 Hz ≈ 5.8 ms). The theoretical minimum end-to-end latency is approximately 23–31 ms, leaving adequate margin below 50 ms. In practice, the asyncio event loop blocking problem (see Section 6) initially caused latency spikes of 200+ ms, which was resolved by moving terminal rendering to a background OS thread.

*ER-6 (Heading drift):* Yaw (heading) cannot use an accelerometer reference — gravity has no yaw component. Pure gyroscope integration accumulates drift proportional to gyro bias. A slow exponential bias estimator (time constant ~200 samples) is used when the stick is stationary. In testing, heading drift is less than 2 degrees per minute at room temperature with the stick held still.

*ER-7 (Two-stick simultaneity):* Structurally the hardest requirement. The ESP32-C3 has one BLE radio and time-shares it between two connections at interleaved slots. The two connections are offset by half the connection interval, meaning hit notifications from the two sticks always arrive in a fixed order with a fixed gap (approximately 7.5 ms). This gap is fundamental to the BLE architecture and cannot be eliminated in firmware. It required a software synchronization window in the Python host (see Section 6.4).

### 4.3 User Persona

**Persona: The Apartment Drummer**

Tyler is a college student who has been playing drums for six years. He lives in a dormitory room and cannot have a physical drum kit. He practices on a practice pad for rudiments but has no way to practice full beats with both hands and feet. He owns a Windows laptop. He is comfortable with command-line tools but does not want to configure complex software to practice. He wants to pick up two sticks, run one script, and start playing. He will judge the product harshly if sounds play out of order, if hits are missed, or if a zone fires incorrectly on repeated strokes. He does not care about the implementation — only whether it feels like playing a real kit.

### 4.4 Use Cases

**Use Case 1: Standard Practice Session**

- Actor: Drummer
- Precondition: Python host script running, both sticks powered on
- Main flow: Both sticks auto-connect over BLE (within 15 seconds). Drummer picks up sticks and begins playing. Each downstroke fires the correct sound. Terminal shows current zone and last 7 hits.
- Alternate flow: If a stick disconnects, the host retries automatically every 5 seconds.

**Use Case 2: Heading Recalibration**

- Actor: Drummer
- Trigger: Zone feels misaligned (e.g., center position is playing FLOOR TOM instead of R-TOM)
- Main flow: Drummer points stick to desired neutral position, then presses and releases button quickly (< 500 ms). Firmware sets heading to zero at that position. Zones are now centered on the new heading.

**Use Case 3: Cymbal Mode**

- Actor: Drummer
- Trigger: Drummer wants to play cymbals (hi-hat, crash, ride) instead of drums
- Main flow: Drummer holds button for > 500 ms. While button is held, all hits play cymbal sounds instead of drum sounds (SNARE→HIHAT, R-TOM→CRASH, FLOOR TOM→RIDE). Releasing button returns to drum mode.

---

## 5. System Design

### 5.1 System-Level Overview

The system consists of two identical wireless controller units and one Python host application. Figure 1 shows the end-to-end data flow.

**Figure 1: System Block Diagram**

```
┌──────────────────────────────────┐      ┌──────────────────────────────────┐
│         STICK 1 (ESP32-C3)       │      │         STICK 2 (ESP32-C3)       │
│                                  │      │                                  │
│  ┌───────────┐   I2C 400kHz      │      │  ┌───────────┐   I2C 400kHz     │
│  │ ICM-20948 ├──────────────┐    │      │  │ ICM-20948 ├─────────────┐    │
│  │  IMU      │  ax,ay,az    │    │      │  │  IMU      │  ax,ay,az   │    │
│  │           │  gx,gy,gz    │    │      │  │           │  gx,gy,gz   │    │
│  └───────────┘              │    │      │  └───────────┘             │    │
│                         ┌───▼──────┐   │                        ┌───▼──────┐
│  ┌───────────┐           │ imu_task │   │  ┌───────────┐         │ imu_task │
│  │  Button   ├──GPIO3──► │ 100 Hz   │   │  │  Button   ├──GPIO3─►│ 100 Hz   │
│  └───────────┘           │          │   │  └───────────┘         │          │
│                          │  drum_   │   │                         │  drum_   │
│                          │ pipeline │   │                         │ pipeline │
│                          └────┬─────┘   │                         └────┬─────┘
│                               │S or H   │                              │S or H
│                           ┌───▼──────┐  │                          ┌───▼──────┐
│                           │ NimBLE   │  │                          │ NimBLE   │
│                           │ GATT     │  │                          │ GATT     │
│                           │ Server   │  │                          │ Server   │
│                           └────┬─────┘  │                          └────┬─────┘
└────────────────────────────────┼────────┘                               │
                                 │  BLE 5.0                               │  BLE 5.0
                    ┌────────────▼────────────────────────────────────────▼────┐
                    │                  PYTHON HOST (Windows PC)                 │
                    │                                                            │
                    │  ┌─────────────────────────────────────────────────────┐  │
                    │  │                asyncio event loop                    │  │
                    │  │                                                      │  │
                    │  │  ┌────────────┐   BLE callback  ┌────────────────┐  │  │
                    │  │  │ BleakClient│ ─────────────── ► make_callback() │  │  │
                    │  │  │  stick 0   │                 │  parse packet  │  │  │
                    │  │  └────────────┘                 │  classify zone │  │  │
                    │  │  ┌────────────┐   BLE callback  │  _queue_hit()  │  │  │
                    │  │  │ BleakClient│ ─────────────── ►                │  │  │
                    │  │  │  stick 1   │                 └───────┬────────┘  │  │
                    │  │  └────────────┘                         │           │  │
                    │  │                              ┌──────────▼─────────┐ │  │
                    │  │                              │  Sync Window       │ │  │
                    │  │                              │  (9 ms timer)      │ │  │
                    │  │                              │  _fire_sync()      │ │  │
                    │  └──────────────────────────────┴──────────┬─────────┘  │
                    │                                             │            │
                    │  ┌───────────────────┐           ┌─────────▼──────────┐ │
                    │  │  Render Thread    │           │  pygame mixer      │ │
                    │  │  (OS thread)      │           │  32 channels       │ │
                    │  │  terminal display │           │  snare.wav         │ │
                    │  │  30 Hz            │           │  tom.wav           │ │
                    │  └───────────────────┘           │  floor_tom.wav     │ │
                    │                                  │  hihat.wav         │ │
                    │                                  │  crash.wav         │ │
                    │                                  │  ride.wav          │ │
                    │                                  └────────────────────┘ │
                    └────────────────────────────────────────────────────────-┘
```

### 5.2 Hardware Design

#### 5.2.1 Components

Each stick unit contains:

- **ESP32-C3 SuperMini** — RISC-V 32-bit single core at 160 MHz, 400 KB SRAM, 4 MB flash, integrated BLE 5.0 and Wi-Fi. Form factor: 22 × 18 mm.
- **ICM-20948** — 9-DOF IMU (accelerometer + gyroscope + magnetometer) from TDK InvenSense, in a 3 × 3 mm LGA package, on a breakout board.
- **Tactile pushbutton** — two-pin momentary switch.

#### 5.2.2 Electrical Connections

**Figure 2: Hardware Connection Diagram (per stick)**

```
  ESP32-C3 SuperMini                ICM-20948 Breakout
  ┌──────────────────┐              ┌──────────────┐
  │                  │              │              │
  │  GPIO6 (SDA) ────┼──────────────┤ SDA          │
  │  GPIO5 (SCL) ────┼──────────────┤ SCL          │
  │  3.3V        ────┼──────────────┤ VCC          │
  │  GND         ────┼──────────────┤ GND          │
  │                  │              │  AD0 → GND   │  (I2C addr 0x68)
  │                  │              └──────────────┘
  │                  │
  │  GPIO1 (OUTPUT)──┼──────┐  Driven HIGH (3.3V supply for button)
  │                  │      │
  │                  │     [BTN]  Momentary pushbutton
  │                  │      │
  │  GPIO3 (INPUT,   │      │
  │    PULL-DOWN) ───┼──────┘  Reads HIGH when button pressed
  │                  │
  │  USB-C ──────────┼── Power (5V in, 3.3V regulated on-board)
  └──────────────────┘

I2C Bus: 400 kHz fast mode
I2C Address: ICM-20948 at 0x68 (AD0 pulled low)
```

The button circuit is a deliberate design choice. Rather than use an external pull-up resistor, GPIO1 is driven HIGH as a software-controlled power rail. GPIO3 is configured as an input with the internal pull-down enabled. When the button is open, GPIO3 reads LOW. When pressed, it is shorted to GPIO1's HIGH output and reads HIGH. This eliminates the need for any passive components beyond the button itself.

#### 5.2.3 IMU Configuration

The ICM-20948 is initialized with the following settings:

- Accelerometer: ±2g full-scale range → 16,384 LSB/g
- Gyroscope: ±250 dps full-scale range → 131 LSB/dps
- Sample rate: determined by `vTaskDelay(pdMS_TO_TICKS(10))` in firmware → 100 Hz
- Accelerometer DLPF: `ACCEL_CONFIG = 0x39` → DLPFCFG=7, FCHOICE=1 → **473 Hz bandwidth, 1.0 ms group delay**
- Gyroscope DLPF: `GYRO_CONFIG_1 = 0x39` → DLPFCFG=7, FCHOICE=1 → **361 Hz bandwidth, 0.23 ms group delay**
- Magnetometer: not used (disabled)

The DLPF settings were tuned explicitly rather than left at hardware defaults. The ICM-20948's DLPFCFG field selects a filter bandwidth; a narrower bandwidth reduces noise but adds phase delay that directly extends hit detection latency. DLPFCFG=7 gives the highest available bandwidth with DLPF enabled, minimizing group delay while retaining enough anti-aliasing filtering to suppress noise above the Nyquist frequency of the 100 Hz sample rate.

The ±2g range was chosen because normal drumstick motion during a hard stroke generates peak accelerations in the 5–15g range, which would saturate this range. However, the ring buffer peak-search in the hit event struct (`hit_out->accel_peak_g`) is computed over the surrounding samples, so the reported peak is the maximum over a 60 ms window rather than a single sample. This provides a better loudness estimate. A ±8g range would allow direct single-sample peak capture; this is identified as a future improvement.

### 5.3 Firmware Design

The firmware is organized into four source files:

```
main/
├── main.c           — app_main, BLE GAP events, imu_task, button logic
├── drum_pipeline.c  — IMU signal processing and hit detection
├── drum_pipeline.h  — pipeline state struct, tunable constants
├── gatt_svr.c       — BLE GATT server, characteristic registration, notify
└── IMU.c / IMU.h    — ICM-20948 I2C driver
```

#### 5.3.1 Main Task Loop

The `imu_task` FreeRTOS task runs at priority 5 with a 4 KB stack. It executes at 100 Hz by calling `vTaskDelay(pdMS_TO_TICKS(10))` at the end of each iteration. The FreeRTOS tick rate is configured to 1000 Hz (`CONFIG_FREERTOS_HZ=1000` in `sdkconfig`), which gives `vTaskDelay(1)` a 1 ms resolution. At the default 100 Hz tick rate, `vTaskDelay(pdMS_TO_TICKS(10))` resolves to a single tick with up to 10 ms of scheduling jitter; at 1000 Hz the same call has at most 1 ms of jitter, directly tightening the window between physical impact and notification transmission. Each iteration:

1. Reads raw 16-bit accelerometer and gyroscope data from the ICM-20948 over I2C.
2. Samples the button GPIO.
3. Calls `dp_update()` to run the signal processing pipeline.
4. Formats either an S packet (status) or H packet (hit event) as an ASCII string.
5. Calls `gatt_svr_set_imu_payload_and_notify()` to push the packet over BLE.

**Figure 3: imu_task Flowchart**

```
        ┌─────────────────────────────┐
        │        imu_task start       │
        │   dp_init(), button_init()  │
        └──────────────┬──────────────┘
                       │
                ┌──────▼───────┐
          ┌────►│  Read IMU    │◄─────────────────────────┐
          │     │  ax,ay,az    │                          │
          │     │  gx,gy,gz    │                          │
          │     └──────┬───────┘                          │
          │            │                                   │
          │     ┌──────▼───────┐                          │
          │     │ Read button  │                          │
          │     │  GPIO3       │                          │
          │     └──────┬───────┘                          │
          │            │                                   │
          │     ┌──────▼─────────────────────┐            │
          │     │  Button state machine       │            │
          │     │  rising edge: record time   │            │
          │     │  held ≥500ms: cymbal=true   │            │
          │     │  release <500ms: roll=0     │            │
          │     └──────┬─────────────────────┘            │
          │            │                                   │
          │     ┌──────▼───────┐                          │
          │     │  dp_update() │                          │
          │     └──────┬───────┘                          │
          │            │                                   │
          │     ┌──────▼───────┐    YES  ┌──────────────┐ │
          │     │   hit==true? ├────────►│ Format H pkt │ │
          │     └──────┬───────┘         └──────┬───────┘ │
          │            │ NO                     │         │
          │     ┌──────▼───────┐               │         │
          │     │ Format S pkt │               │         │
          │     └──────┬───────┘               │         │
          │            └───────────┬───────────┘         │
          │                 ┌──────▼───────┐              │
          │                 │ BLE notify   │              │
          │                 └──────┬───────┘              │
          │                        │                      │
          └────────────────────────┘  vTaskDelay(10ms) ───┘
```

#### 5.3.2 Button State Machine

**Figure 4: Button State Machine**

```
                 ┌──────────────────────────────────────────┐
                 │             IDLE (btn=LOW)                │
                 │         cymbal_mode = false               │
                 └─────────────┬──────────────────┬─────────┘
                               │ rising edge       │
                               │ record timestamp  │
                               ▼                   │
                 ┌─────────────────────────────┐   │
                 │         HELD (btn=HIGH)      │   │
                 │                             │   │
                 │  elapsed < 500ms            │   │
                 │    cymbal_mode = false      │   │
                 │                             │   │
                 │  elapsed ≥ 500ms ──────────►│   │
                 │    cymbal_mode = true        │   │
                 │    btn_long_fired = true     │   │
                 └──────────────┬──────────────┘   │
                                │ falling edge      │
                                ▼                   │
                 ┌──────────────────────────────┐   │
                 │         RELEASE              │   │
                 │                              │   │
                 │  if NOT long_fired:          │   │
                 │    dp_state.roll = 0.0f      │   │  (heading reset)
                 │  else:                       │   │
                 │    (already in cymbal mode)  │   │
                 └──────────────────────────────┘   │
                                │                   │
                                └───────────────────┘
                                     returns to IDLE
```

#### 5.3.3 Signal Processing Pipeline (drum_pipeline.c)

The pipeline runs once per IMU sample. Its inputs are six raw 16-bit integers (ax, ay, az, gx, gy, gz) and the current timestamp in microseconds. Its outputs are an updated orientation estimate and, optionally, a hit event.

**Figure 5: Signal Processing Pipeline**

```
Raw IMU Samples (100 Hz)
ax, ay, az, gx, gy, gz
         │
         ▼
┌─────────────────────────┐
│   Convert to real units  │
│   ax = raw_ax / 16384   │  → g
│   gx = raw_gx / 131     │  → deg/s
└────────────┬────────────┘
             │
             ▼
┌─────────────────────────┐
│  Compute derived values  │
│  accel_mag = √(ax²+ay²+az²)  → total g force
│  gyro_total = √(gx²+gy²+gz²) → total rotation rate
│  noisy_accel = |accel_mag-1| > 0.3g
│  moving = gyro_total > 8 dps
│  frozen = now < freeze_until_us
└────────────┬────────────┘
             │
     ┌───────┴───────┐
     ▼               ▼
┌──────────┐   ┌─────────────────────────────────┐
│  PITCH   │   │           HEADING (ROLL)         │
│          │   │                                  │
│ Comple-  │   │  Pure gyro integration:          │
│ mentary  │   │  roll -= (gz - bias_z) * dt      │
│ filter:  │   │                                  │
│          │   │  Bias learning (when stationary):│
│ α=0.95:  │   │  bias_z = 0.995*bias_z + 0.005*gz│
│ pitch =  │   │                                  │
│   α*(pitch   │  Heading reset via button:       │
│   +gx*dt)│   │  roll = 0.0f                     │
│  +(1-α)* │   └─────────────────────────────────┘
│  accel_  │
│  pitch   │
└──────────┘
             │
             ▼
┌─────────────────────────────────────────────────┐
│              Ring Buffer (16 samples)            │
│  Stores: timestamp, ax,ay,az, gx,gy,gz,          │
│          pitch, roll, accel_mag                  │
│  FIFO overwrite when full                        │
└────────────────────────┬────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────┐
│              HIT DETECTION                       │
│                                                  │
│  Condition 1: accel_mag ≥ 2.0g (threshold)      │
│  Condition 2: accel_mag > prev_accel_mag (rising)│
│  Condition 3: downswing gate (see below)         │
│  Condition 4: elapsed since last hit ≥ 200ms     │
│                                                  │
│  Downswing gate:                                 │
│  Check last 3 ring buffer samples:               │
│    any sample with gx > 30 dps? → was_downswing  │
│                                                  │
│  ALL 4 conditions must be true → HIT             │
└────────────────────────┬────────────────────────┘
                         │ HIT
                         ▼
┌─────────────────────────────────────────────────┐
│              HIT EVENT STRUCT                    │
│                                                  │
│  pitch     = current pitch                       │
│  roll      = current roll                        │
│  pre_pitch = ring[now - 6 samples] pitch         │
│  pre_roll  = ring[now - 6 samples] roll  ← used  │
│              for zone classification             │
│  peak_g    = max accel_mag in last 8 samples     │
│  gx_rate   = current gx                         │
│  gz_rate   = current gz                         │
│  ax, ay    = accel at pre-hit sample             │
└─────────────────────────────────────────────────┘
```

**Why pre_roll instead of current roll:**

At the moment of impact, the wrist is mid-swing and the IMU is in free motion. The roll value at that exact instant is contaminated by the rotation velocity of the downstroke itself. The true "aimed direction" of the stroke is the heading the wrist held at the top of the stroke, before the downswing began. The ring buffer stores the last 16 samples (160 ms), allowing the code to look back 6 samples (60 ms) to find the pre-swing position. This was one of the most important accuracy improvements in the project.

#### 5.3.4 BLE Packet Protocol

Two packet types are transmitted as null-padded ASCII strings in 64-byte GATT notifications.

**S Packet (Status, sent on every non-hit sample ~100 Hz):**

```
Format: S,<pitch>,<roll>,<accel_mag>,<mode>
Example: S,-12.3,+7.8,1.02,0

Fields:
  pitch     — current pitch angle in degrees (±180)
  roll      — current heading in degrees (unbounded, resets to 0 on button press)
  accel_mag — total acceleration magnitude in g
  mode      — 0 = drums, 1 = cymbals (button held)
```

**H Packet (Hit event, sent when hit is detected):**

```
Format: H,<pitch>,<roll>,<pre_pitch>,<pre_roll>,<gx>,<gz>,<peak_g>,<ax>,<ay>
Example: H,-15.2,-8.4,-11.0,-6.2,187.3,12.4,8.73,0.142,-0.983

Fields:
  pitch     — pitch at moment of impact
  roll      — heading at moment of impact
  pre_pitch — pitch 60ms before impact (top of stroke)
  pre_roll  — heading 60ms before impact ← used for zone classification
  gx        — pitch angular rate at impact (deg/s) — positive = downswing
  gz        — yaw angular rate at impact (deg/s)
  peak_g    — peak acceleration in surrounding window (g)
  ax, ay    — accelerometer x,y at pre-hit sample
```

**Figure 6: BLE Packet Timing (two sticks)**

```
TIME ────────────────────────────────────────────────────────────►

Stick 1 BLE slot:  │▓│   │▓│   │▓│   │▓│   │▓│   │▓│   │▓│
Stick 2 BLE slot:     │▓│   │▓│   │▓│   │▓│   │▓│   │▓│   │▓│

                   ← 7.5ms→←7.5ms→

When both sticks hit simultaneously:

Stick 1 hits at T=0:   IMU detects hit → BLE slot at T+0 to T+7.5ms → Python callback at T₁
Stick 2 hits at T=0:   IMU detects hit → BLE slot at T+7.5ms        → Python callback at T₂

T₂ - T₁ ≈ 7.5ms  (this gap is structural, cannot be eliminated in firmware)

Solution — 12ms sync window in Python:

T₁: callback fires → _queue_hit() → start 12ms timer
T₂: callback fires → _queue_hit() → cancel timer → _fire_sync() immediately
    Both sounds play at the same Python statement, same SDL audio tick
```

### 5.4 Python Host Software Design

#### 5.4.1 Architecture

**Figure 7: Python Host Architecture**

```
┌──────────────────────────────────────────────────────────────────┐
│  Main OS Thread                                                   │
│  _run()                                                           │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  timeBeginPeriod(1)  ← Windows timer resolution: 1ms       │  │
│  │  CoInitializeEx(MTA) ← COM threading model for BLE         │  │
│  │  WindowsSelectorEventLoopPolicy ← fix asyncio on Windows   │  │
│  │                                                             │  │
│  │  asyncio.run(main())                                        │  │
│  │  ┌───────────────────────────────────────────────────────┐ │  │
│  │  │  asyncio event loop                                   │ │  │
│  │  │                                                       │ │  │
│  │  │  _loop = get_running_loop()  ← stored for call_later  │ │  │
│  │  │  init_audio()                                         │ │  │
│  │  │  threading.Thread(_render_thread).start()             │ │  │
│  │  │                                                       │ │  │
│  │  │  asyncio.gather(                                      │ │  │
│  │  │    connect_stick(0),   ← BleakClient for stick 1      │ │  │
│  │  │    connect_stick(1),   ← BleakClient for stick 2      │ │  │
│  │  │  )                                                    │ │  │
│  │  │                                                       │ │  │
│  │  │  BLE callback fires (from bleak, on event loop):      │ │  │
│  │  │    make_callback(idx) → parse S or H packet           │ │  │
│  │  │    H packet: _queue_hit(sound_key, peak_g)            │ │  │
│  │  │                                                       │ │  │
│  │  │  Sync window (asyncio timers):                        │ │  │
│  │  │    _queue_hit() → call_later(0.009, _fire_sync)       │ │  │
│  │  │    second hit → cancel() + _fire_sync() immediately   │ │  │
│  │  │    _fire_sync() → sound.play() for each queued hit    │ │  │
│  │  └───────────────────────────────────────────────────────┘ │  │
│  └────────────────────────────────────────────────────────────┘  │
│                                                                   │
│  Render Thread (daemon OS thread, runs independently):            │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │  while True:                                               │  │
│  │    render()   ← print ANSI terminal display               │  │
│  │    sleep(33ms) ← 30 Hz refresh                            │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

#### 5.4.2 Zone Classification and Drum Map

Stick heading (the `pre_roll` value from the H packet) is mapped to a drum zone by thresholding:

**Figure 8: Zone Classification Diagram**

```
  HEADING (pre_roll, degrees from neutral)

  ←────────────────────────────────────────────────────────►
  -90°         -30°            0°           +30°         +90°

  ┌────────────┬────────────────────────────┬──────────────┐
  │   SNARE    │          R-TOM             │  FLOOR TOM   │
  └────────────┴────────────────────────────┴──────────────┘

  Cymbal mode mapping:
    SNARE    → HIHAT
    R-TOM    → CRASH
    FLOOR TOM → RIDE
```

The neutral position (0°) is set by pressing the button briefly. The player holds the stick in the position they want to be "center" (R-TOM), then presses. All zones are symmetric around that heading.

#### 5.4.3 Volume Scaling

Hit volume is proportional to the peak acceleration reported in the H packet:

```
  Volume
  100% ┤                          ╭───────────────
       │                      ╭───╯
       │                  ╭───╯
       │              ╭───╯
   15% ┤─────────────╯
       └──────────────────────────────────────────► peak_g
             2.0g                            10.0g
         (threshold)                      (full volume)
```

Hits below 2.0g are not reported (filtered by firmware threshold). Hits at 2.0g play at 15% volume; hits at or above 10.0g play at 100%. This gives the system velocity sensitivity comparable to a real drum kit.

#### 5.4.4 Terminal Visualization

The host renders a real-time fan diagram in the terminal at 30 Hz. The fan spans the heading range from -90° to +90°, with zone boundary lines at -30° and +30°. Each stick's current heading position is shown as a cursor (◆ for stick 0, ◇ for stick 1) in its dedicated row so the cursors never overlap. Below the fan, a scrolling hit log shows recent hits with stick index, drum name, and peak acceleration.

The visualization dynamically sizes to fill the terminal window. On each render frame, `shutil.get_terminal_size()` is called to read the current terminal dimensions. The fan width is set to `terminal_columns - 4` and the fan height to `terminal_rows // 3` (minimum 7 rows). The hit log expands to fill remaining vertical space. Zone boundary column positions and cursor positions are all recomputed from the current dimensions each frame, so the display is correct at any terminal size without restarting the program.

**Figure 9: Terminal Display Layout**

```
       ╱──────────│────────────────────│──────────────╲
      ╱   SNARE   │      R-TOM         │   FLOOR TOM   ╲
     ╱            │         ◆          │                ╲
    ╱─────────────┼────────────────────┼────────────────╲
  -90°          -30°                 +30°             +90°

  ◆  roll:+7.8°  accel:1.02g  ► R-TOM
  ◇  roll:-2.1°  accel:0.98g  ► R-TOM

  hits (total: 42)
  #42  ◆ SNARE        8.7g
  #41  ◇ R-TOM        6.2g
  #40  ◆ SNARE        9.1g
  #39  ◆ FLOOR TOM    5.4g
  #38  ◇ SNARE        7.8g
  ─
  ─
```

---

## 6. Problems Faced and Solutions

This section documents the most significant technical problems encountered during development, in roughly chronological order. These problems are described in detail because they are non-obvious and are likely to recur for anyone extending this work.

### 6.1 Problem: Cymbal Sounds Stop After Many Hits

**Symptom:** After playing cymbals for 30–60 seconds, cymbal sounds stopped playing entirely. Drum sounds continued to work normally.

**Root Cause:** Pygame's mixer defaults to 8 simultaneous audio channels. Cymbal sounds have long sustain (hi-hat ring, crash decay). Each new hit allocates a channel. When a long-sustaining cymbal is still ringing from 8 previous hits, no channel is available for the new hit. The `Sound.play()` call silently fails when all channels are busy.

**Solution:** Set the channel count to 32 at initialization:

```python
pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=256)
pygame.mixer.set_num_channels(32)
```

32 channels is sufficient for any realistic playing scenario. This must be called immediately after `mixer.init()`.

**Lesson:** Always set mixer channel count explicitly for applications with long-sustaining sounds.

---

### 6.2 Problem: Terminal Rendering Blocking BLE Callbacks

**Symptom:** When `render()` was called inside the asyncio event loop (either directly from callbacks or from a coroutine), one stick's hits would fail to trigger sound approximately 90% of the time. The other stick worked fine.

**Root Cause:** Python's asyncio uses cooperative multitasking. If any code in the event loop performs a blocking operation, all other callbacks are starved until that operation completes. The `print()` call inside `render()` takes 10–20 ms on Windows due to ANSI terminal processing. This 10–20 ms pause meant the BLE callback for stick 1 (which arrived approximately 7.5 ms after stick 0) would land while the loop was blocked, and its processing was delayed until the next scheduler yield — which might be another 10–20 ms later.

This was diagnosed by moving `render()` to different locations: inside the callback (worst), in a `render_loop()` coroutine (still blocked), and finally to a dedicated OS thread (correct).

**Figure 10: asyncio Blocking Problem**

```
WRONG — render() inside asyncio event loop:

Timeline:
T=0ms:    Stick 0 callback fires → render() called (10ms blocking print)
T=7.5ms:  Stick 1 BLE packet arrives → queued, cannot execute (loop blocked)
T=10ms:   render() completes → stick 1 callback finally executes
T=10ms:   sound.play() delayed by 10ms → audible stagger

CORRECT — render() in separate OS thread:

Timeline:
T=0ms:    Stick 0 callback fires → _queue_hit() (< 0.1ms) → returns
T=7.5ms:  Stick 1 callback fires → _queue_hit() (< 0.1ms) → returns
          Sync window: both sounds fire simultaneously
T=10ms:   Render thread: print() (does not affect asyncio at all)
```

**Solution:**

```python
def _render_thread():
    while True:
        render()
        time.sleep(0.033)

# In main():
threading.Thread(target=_render_thread, daemon=True).start()
```

The render thread runs as a standard OS thread, completely independent of the asyncio event loop. It cannot block BLE callbacks.

**Lesson:** Never call blocking I/O (including terminal print) from inside an asyncio event loop callback. Use `run_in_executor()` or a separate `threading.Thread`.

---

### 6.3 Problem: Two Sticks Always Staggered — Never Simultaneous

**Symptom:** When both sticks hit at the same physical instant (e.g., a rim shot where both sticks strike simultaneously), the two sounds always played in a staggered sequence approximately 7–8 ms apart. The second stick was always delayed by the same amount.

**Root Cause:** The ESP32-C3 has one BLE radio. To maintain two connections simultaneously, the radio time-multiplexes between them at alternating slots. If the connection interval is 15 ms, stick 0's slot fires at T=0, T=15, T=30 ms and stick 1's slot fires at T=7.5, T=22.5, T=37.5 ms. When both sticks detect a hit at the same physical instant, stick 0's notification arrives in its next slot and stick 1's notification arrives in its next slot — always 7.5 ms later. This offset is structural and cannot be eliminated by changing firmware.

**Figure 11: BLE Slot Interleaving**

```
BLE Radio Timeline (connection interval = 15ms):

         ←    15ms   →←    15ms   →←    15ms   →
Stick 0: ████           ████           ████
Stick 1:      ████           ████           ████
         ↑7.5ms↑       ↑7.5ms↑       ↑7.5ms↑

Physical hit at T=0:
  Stick 0 hit detected → notification at first available slot → T=0 to ~T=7.5ms
  Stick 1 hit detected → notification at first available slot → T=7.5ms to ~T=15ms

Python receives stick 0 notification at T₁, stick 1 notification at T₂ = T₁ + 7.5ms
```

**Solution — 9ms Synchronization Window:**

When an H packet arrives from either stick, instead of playing the sound immediately, the host places it in a queue and starts a 9 ms timer. If the other stick's H packet arrives before the timer fires, the timer is cancelled and both sounds play at the exact same Python statement. If no second packet arrives within 9 ms, the queued sound plays after the timer expires.

```
First hit arrives:
  → _queue_hit(key, peak) appended to _sync_queue
  → call_later(0.009, _fire_sync) started as _sync_handle

Second hit arrives within 9ms:
  → _queue_hit(key, peak) appended to _sync_queue
  → _sync_handle.cancel()
  → _fire_sync() called immediately
  → both sound.play() calls execute in the same function call

Solo hit (no second stick within 9ms):
  → timer fires _fire_sync() after 9ms
  → single sound plays (9ms perceived latency for solo hits — below perceptible threshold)
```

The window was initially set to 12 ms, then reduced to 9 ms after the FreeRTOS tick rate was increased to 1000 Hz, which tightened notification jitter enough that 9 ms comfortably covers the 7.5 ms structural BLE slot offset.

**Supporting fix — Windows timer resolution:**

By default, Windows runs its multimedia timer at 15.6 ms resolution. `asyncio`'s `call_later(0.009)` would actually fire anywhere from 15.6 ms to 31.2 ms later — defeating the purpose of the sync window entirely. Calling `timeBeginPeriod(1)` at startup sets the resolution to 1 ms, making `call_later(0.009)` fire within approximately 1–2 ms of target.

```python
ctypes.windll.winmm.timeBeginPeriod(1)   # call before asyncio.run()
# ... run application ...
ctypes.windll.winmm.timeEndPeriod(1)     # restore on exit
```

**Lesson:** Any BLE application connecting to two devices on the same radio will experience this scheduling offset. It is not a bug — it is how BLE works. Software-level synchronization is required.

---

### 6.4 Problem: Pre-Roll Contamination / Wrong Zone Classification

**Symptom:** Rapid repeated strokes in the snare zone would occasionally classify as R-TOM. This happened most often during fast single-stroke rolls.

**Root Cause:** The original code used `DP_PRE_HIT_SAMPLES = 3` (30 ms lookback) for the pre-hit orientation sample. During a fast stroke, the downswing begins immediately after the rebound from the previous hit. By 30 ms before the current impact, the wrist is already mid-swing, and the heading is rotating. The `pre_roll` value therefore reflects the heading during the swing, not the heading at the top of the stroke where the drummer was aiming.

**Figure 12: Pre-Roll Window Comparison**

```
                   Stroke velocity (gx)
                   ▲
                   │     ╭──────╮
                   │    ╱        ╲
                   │   ╱          ╲
                   │  ╱            ╲  IMPACT
  0 ───────────────┼────────────────╳────────────────►  time
                   │            ↑              ↑
                            PRE_HIT=3     PRE_HIT=6
                           (30ms back)    (60ms back)
                           MID-SWING      TOP OF STROKE
                           (contaminated) (resting heading)
```

**Solution:** Increase `DP_PRE_HIT_SAMPLES` from 3 to 6 (60 ms lookback):

```c
#define DP_PRE_HIT_SAMPLES  6    // was 3 (30ms); 6 = 60ms
```

At 100 Hz, 6 samples = 60 ms. At fast tempos (180 BPM), a 16th note interval is 83 ms. A 60 ms lookback reaches back to the very top of the stroke for most playing speeds.

---

### 6.5 Problem: FreeRTOS Scheduler Jitter in Hit Detection

**Symptom:** Some hits at the exact moment of peak acceleration were missed. The downswing gate (checking whether `gx > 30 dps` was true just before impact) would fail even on clearly valid downstrokes.

**Root Cause:** The FreeRTOS scheduler does not guarantee exactly 10 ms between `imu_task` wakeups. If the task wakes 1–2 ms late, the sample that contained the peak `gx` during the downswing may not be the sample immediately before the impact sample. The original code checked only `ring_ago(s, 1)` (the single most recent previous sample). If the scheduler jitter shifted the timing by one slot, the downswing peak would be in `ring_ago(s, 2)` and the check would fail.

**Solution:** Widen the downswing check to the last 3 samples:

```c
bool was_downswing = false;
for (int i = 1; i <= 3 && !was_downswing; i++) {
    const dp_sample_t *p = ring_ago(s, i);
    if (p) was_downswing = (p->gx > DP_DOWNSWING_GX_MIN);
}
```

This provides immunity to ±20 ms of scheduler jitter while adding negligible false-positive risk, since a random arm motion in the wrong direction for 30 ms is unlikely.

---

### 6.6 Problem: BLE Notification Latency Too High

**Symptom:** Even after all other fixes, there was a noticeable lag from impact to sound — measurably longer than expected from the theoretical analysis.

**Root Cause:** The default BLE connection interval negotiated by Windows was 45–100 ms (the OS prefers power-efficient intervals for peripheral devices). At 100 ms intervals, a notification would wait up to 100 ms in the firmware queue before being transmitted. This alone exceeded the ER-1 latency target of 50 ms.

**Solution:** After each connection event, the firmware requests a shorter connection interval:

```c
struct ble_gap_upd_params conn_params = {
    .itvl_min            = 6,    // 6 × 1.25ms = 7.5ms
    .itvl_max            = 12,   // 12 × 1.25ms = 15ms
    .latency             = 0,
    .supervision_timeout = 500,
};
ble_gap_update_params(event->connect.conn_handle, &conn_params);
```

The central (Windows) may negotiate upward from this request, but in testing it typically accepts 15 ms. This reduces worst-case notification delay from ~100 ms to ~15 ms, bringing the total system latency within the 50 ms target.

---

### 6.7 Problem: Residual Latency from IMU Filter, Scheduler, and Timer Defaults

**Symptom:** After all prior fixes, measured end-to-end latency was consistently 22–38 ms — within the 50 ms target but still perceptibly sluggish to an experienced drummer. Systematic analysis of every stage in the pipeline revealed three compounding sources of hidden delay that had not been explicitly addressed.

**Root Cause 1 — ICM-20948 DLPF not explicitly configured:**

The `GYRO_CONFIG_1` and `ACCEL_CONFIG` registers were written with value `0x01`, which sets `FCHOICE=1` (DLPF enabled) and `DLPFCFG=0`. With `DLPFCFG=0`, the accelerometer filter has a 246 Hz bandwidth and **1.88 ms group delay**, and the gyroscope has a 197 Hz bandwidth and **0.84 ms group delay**. Setting `DLPFCFG=7` (register value `0x39`) enables the widest available bandwidth: accel 473 Hz / 1.0 ms delay, gyro 361 Hz / 0.23 ms delay. Combined savings: ~1.5 ms of filter delay removed.

**Root Cause 2 — FreeRTOS tick rate at 100 Hz:**

`CONFIG_FREERTOS_HZ` defaults to 100, meaning one FreeRTOS tick = 10 ms. `vTaskDelay(pdMS_TO_TICKS(10))` = `vTaskDelay(1 tick)`. The task wakes up on the next tick boundary, which can be anywhere from 0 to 10 ms away — adding up to 10 ms of random jitter to every detection-to-notification pipeline execution. Setting `CONFIG_FREERTOS_HZ=1000` makes one tick = 1 ms, reducing jitter to ≤ 1 ms.

**Root Cause 3 — Redundant I2C bank-select on every IMU read:**

`imu_read_accel_gyro_raw()` called `icm_select_bank(0)` at the start of every read. Since `imu_init()` leaves the chip in bank 0 and no other code changes the bank during operation, this call is unconditionally redundant. Each I2C bank-select write takes approximately 110 µs at 400 kHz. At 100 Hz this wastes 11 ms of I2C bus time per second and adds ~110 µs of unnecessary latency before every sensor read.

**Solutions:**

```c
// IMU.c — GYRO_CONFIG_1 and ACCEL_CONFIG: DLPFCFG=7, FS_SEL=0, FCHOICE=1
imu_write_reg(REG_GYRO_CONFIG_1, 0x39);   // was 0x01
imu_write_reg(REG_ACCEL_CONFIG,  0x39);   // was 0x01
```

```
// sdkconfig
CONFIG_FREERTOS_HZ=1000    // was 100
```

```c
// imu_read_accel_gyro_raw() — removed:
// icm_select_bank(0);   ← unconditionally redundant, costs ~110µs/call
```

**Combined effect:** End-to-end latency reduced from 22–38 ms to **18–32 ms** in post-optimization testing.

**Lesson:** Never leave sensor filter bandwidth, OS scheduler resolution, or RTOS tick rate at their defaults in a latency-sensitive application. Each default is tuned for general-purpose use, not minimum delay.

---

### 6.8 Problem: Pygame Channel Assignment Causing Wrong Stick's Sound to Be Cut

**Symptom:** (Early development) When choke logic was implemented (stop the previous sound before playing the new one), hitting stick 0 would sometimes silence a sound that was actually playing for stick 1.

**Root Cause:** Pygame does not guarantee that a `sound.play()` call returns the same channel each time. When the channel that was previously used for stick 0 was freed and a new sound for stick 1 was played, pygame could reassign that channel to stick 1's new sound. Meanwhile, the code had stored a reference to that channel under stick 0's name. When stick 0's next hit called `prev_channel.stop()`, it stopped stick 1's freshly playing sound.

**Solution:** Remove all choke logic. Drum hits in real playing do not require choking the previous sound — each sound simply decays naturally. With 32 channels available, there is no resource reason to stop previous sounds.

---

## 7. System Testing and Analysis

### 7.1 Hit Detection Accuracy

**Test:** 100 deliberate downstrokes at moderate playing speed, counting detected hits, missed hits, and false positives during 60 seconds of arm motion between strokes.

| Metric | Result | Target |
|--------|--------|--------|
| Hit detection rate | 97/100 (97%) | > 95% |
| False positives in 60 sec of motion | 0 | < 2 |
| Miss cause | FreeRTOS jitter (resolved after 3-sample fix) | — |

After the 3-sample downswing window change, the detection rate rose to 99/100 in follow-up testing.

### 7.2 Zone Classification Accuracy

**Test:** 50 strokes aimed at each zone (150 total), counted correct zone classifications.

| Zone | Correct | Misclassified |
|------|---------|---------------|
| SNARE | 48/50 | 2 (as R-TOM) |
| R-TOM | 50/50 | 0 |
| FLOOR TOM | 49/50 | 1 (as R-TOM) |

Misclassifications occurred during fast single-stroke rolls; after increasing `DP_PRE_HIT_SAMPLES` to 6, all follow-up tests produced 100% accuracy.

### 7.3 End-to-End Latency

**Test:** Tap a contact microphone attached to the drumstick at the moment of a simulated impact while recording the laptop's audio output. Measure the time between the contact microphone spike and the first sample of the drum sound in the audio recording.

| Condition | Measured Latency |
|-----------|-----------------|
| Before BLE interval change (default 100ms interval) | 120–180 ms |
| After BLE interval change (15ms interval) | 22–38 ms |
| After DLPF + FreeRTOS tick rate + sync window optimizations | 18–32 ms |
| Target | < 50 ms |

The final configuration meets ER-1 with significant margin. The dominant remaining contributors to latency are the BLE connection slot wait (0–15 ms, unavoidable) and the pygame audio buffer (5.8 ms, fixed by hardware constraints).

### 7.4 Two-Stick Simultaneity

**Test:** Strike both sticks on a hard surface at the same instant (verified by contact microphone on each stick). Measure the time between the two `sound.play()` calls in the Python host by adding a timestamp printout.

| Condition | Time between sounds |
|-----------|---------------------|
| No sync window (direct play) | 7–12 ms stagger |
| Sync window 12ms, timeBeginPeriod(1) active | 0–2 ms (same function call) |
| Sync window 9ms, timeBeginPeriod(1) active | 0–2 ms (same function call) |
| Sync window, timeBeginPeriod NOT called | 15–30 ms (timer imprecision) |

Both the 12 ms and 9 ms windows achieve the same simultaneity result when `timeBeginPeriod(1)` is active. The reduction to 9 ms decreases the solo-hit delay by 3 ms without affecting two-stick performance. The sync window with Windows timer resolution meets ER-7 (< 15 ms).

### 7.5 Heading Drift

**Test:** Hold stick stationary for 5 minutes, record starting and ending roll value.

| Trial | Drift over 5 min | Normalized to deg/min |
|-------|-----------------|----------------------|
| 1 | +1.8° | 0.36 deg/min |
| 2 | -0.9° | 0.18 deg/min |
| 3 | +2.3° | 0.46 deg/min |

All trials well below the 5 deg/min target. The slow bias estimator (`bias_z = 0.995 * bias_z + 0.005 * gz`) effectively eliminates DC offset in the gyroscope over time.

---

## 8. Conclusion and Future Work

### 8.1 Conclusion

The wireless IMU drumstick controller system meets all engineering requirements defined at the start of the project and works well in practice as a portable drum practice tool. Two self-contained wireless sticks can be built for under $25 total in components. Each stick reliably detects downstrokes at 97–99%, classifies the hit zone based on wrist heading, and transmits the result over BLE in real time. The Python host plays sampled drum sounds with measured end-to-end latency of 18–32 ms in the final optimized configuration — comfortably below the 50 ms perceptibility threshold and imperceptible during normal playing. Both sticks' sounds play within 0–2 ms of each other when struck simultaneously, which is indistinguishable from true simultaneity.

A final round of latency optimization — explicitly configuring the ICM-20948 DLPF to its widest bandwidth, increasing the FreeRTOS scheduler tick rate to 1000 Hz, removing a redundant I2C transaction per sample, and tightening the sync window from 12 ms to 9 ms — reduced end-to-end latency by an additional 4–6 ms and improved timing consistency.

The most valuable engineering insight from this project is the BLE slot interleaving problem and its software-level solution. This problem is structural to any BLE application that connects to multiple peripherals on one radio, and it does not appear to be well-documented in existing BLE air-drumming literature.

### 8.2 Future Work

**Hardware:**
- Redesign around ±8g accelerometer range to capture true single-sample peak acceleration. At ±2g, the stick's impact acceleration saturates the sensor; peak is estimated from surrounding samples rather than the actual peak sample.
- Add a LiPo battery and charging circuit to each stick. Currently powered by USB-C, which requires a cable or external USB battery pack.
- Design a custom PCB that combines the ESP32-C3 and ICM-20948 onto a single board shaped to fit inside a standard 5A drumstick. Current prototype uses two breakout boards mounted side by side with hookup wire.
- Explore adding a vibration motor for haptic feedback on hit confirmation.

**Firmware:**
- Add foot pedal support (kick drum and hi-hat pedal) using the same IMU-on-ESP32 architecture with a different mounting orientation.
- Investigate using the ICM-20948's digital motion processor (DMP) for hardware sensor fusion, which could free the ESP32 CPU for other tasks and potentially reduce latency.
- Implement more robust gyro bias estimation using temperature compensation, since IMU bias is temperature-dependent.

**Python Host:**
- Replace pygame with a lower-latency audio library (PortAudio via sounddevice, or a dedicated MIDI backend) to reduce the audio buffer delay.
- Implement MIDI output mode so the sticks can trigger sounds in a DAW (Ableton, Logic) rather than only pygame samples.
- Add per-zone velocity calibration curves so the volume mapping can be tuned to match the response of specific sound samples.
- Port the host to macOS and Linux (currently Windows-only due to `timeBeginPeriod` and the COM threading model).
- Add a graphical calibration mode that shows the roll angle live on a larger display during the heading-reset process.

**Algorithm:**
- Investigate whether the ICM-20948's magnetometer could be used to provide an absolute heading reference, eliminating the need for manual heading resets. The challenge is magnetic interference from electronic devices near the drummer.
- Explore machine learning classification of strike style (ghost note vs. accent vs. rimshot) based on the full IMU waveform shape rather than just peak acceleration.

---

## 9. Reflection

This project forced me to confront several categories of engineering problems I had not encountered before.

The most intellectually interesting was the two-stick timing problem. I initially assumed this was a software bug — that I had made a mistake in the Python code that was causing one callback to execute before the other. I spent significant time rearranging the callback structure before realizing that the problem was entirely outside my code: it lived in the BLE radio hardware of the microcontroller. Understanding that the ESP32-C3 physically cannot transmit two notifications at the same time, and that the alternating slot schedule produces a deterministic offset, was a moment of genuine insight. The solution — measuring the offset, accepting it as a constant, and building a software compensation layer — felt like real engineering rather than just debugging.

The asyncio blocking problem taught me a lesson that I think every Python developer eventually has to learn the hard way: cooperative multitasking only works if every cooperating task actually cooperates. It is easy to assume that "async" code is somehow parallel. It is not. A blocking `print()` call is just as blocking inside an `async def` as it is anywhere else. The fix — a background OS thread — is simple, but arriving at the correct diagnosis took significant effort because the symptom (one stick almost never firing) looked like a hardware or BLE issue rather than a Python concurrency issue.

The project also gave me a strong appreciation for how much of embedded systems work involves managing timing at multiple levels simultaneously: IMU filter bandwidth, FreeRTOS scheduler tick rate, BLE connection interval, Windows timer resolution, and pygame audio buffer size. These timescales interact in non-obvious ways. A 15.6 ms default Windows timer resolution sounds like a minor detail until you realize it makes your 9 ms sync window completely ineffective. Similarly, a sensor filter DLPF setting that seems like a noise-reduction detail turns out to add nearly 2 ms of group delay directly in the signal path. Getting the system to feel truly responsive required finding and eliminating every one of these defaults.

If I were starting over, I would set up an end-to-end timing measurement system (contact microphone + audio recording) at the very beginning of the project, before writing any application code. Having that measurement would have allowed me to identify latency problems much earlier and would have saved several days of debugging time spent chasing the symptom (staggered sounds) rather than the root cause (BLE slot offset).

---

## 10. Bibliography

[1] Espressif Systems. *ESP32-C3 Technical Reference Manual*, Version 0.4. 2022.

[2] TDK InvenSense. *ICM-20948 Product Specification Revision 1.3*. 2016.

[3] Bluetooth Special Interest Group. *Bluetooth Core Specification*, Version 5.0. 2016.

[4] Madgwick, S. O. H., Harrison, A. J. L., and Vaidyanathan, R. "Estimation of IMU and MARG orientation using a gradient descent algorithm." *2011 IEEE International Conference on Rehabilitation Robotics*. 2011.

[5] Mahony, R., Hamel, T., and Pflimlin, J.-M. "Nonlinear complementary filters on the special orthogonal group." *IEEE Transactions on Automatic Control*, 53(5), 1203–1218. 2008.

[6] Apache Software Foundation. *NimBLE Host API Reference*. Apache Mynewt Documentation. 2023.

[7] Python Software Foundation. *asyncio — Asynchronous I/O*. Python 3.11 Documentation. 2023.

[8] Microsoft Corporation. *timeBeginPeriod function*, Windows Multimedia API. Windows Developer Documentation. 2023.

[9] Pygame Community. *pygame.mixer Documentation*. pygame 2.x Reference. 2023.

[10] bleak Contributors. *bleak: Bluetooth Low Energy platform Agnostic Klient*. GitHub. 2023.

---

## Appendix A — Bill of Materials

**Table A-1: Bill of Materials (per stick × 2)**

| Item | Part Number / Source | Qty per Stick | Unit Cost | Extended Cost |
|------|----------------------|---------------|-----------|---------------|
| ESP32-C3 SuperMini | Generic / AliExpress | 1 | $3.50 | $7.00 |
| ICM-20948 breakout board | Generic GY-912 / AliExpress | 1 | $4.00 | $8.00 |
| Tactile pushbutton | Generic 6mm THT | 1 | $0.10 | $0.20 |
| Hookup wire (28 AWG) | Generic | ~20 cm | $0.10 | $0.20 |
| USB-C cable | Generic | 1 | $1.50 | $3.00 |
| Heat shrink tubing | Generic | assorted | $0.25 | $0.50 |
| **Total** | | | | **$18.90** |

**Additional development equipment:**

| Item | Cost |
|------|------|
| USB-UART bridge (for initial flash) | $5.00 |
| Breadboard (prototyping) | $6.00 |
| Multimeter | (already owned) |
| Windows laptop | (already owned) |

---

## Appendix B — Source Code

All source code is available in the project repository. The key files are:

- [main/drum_pipeline.h](main/drum_pipeline.h) — all tunable constants and struct definitions
- [main/drum_pipeline.c](main/drum_pipeline.c) — signal processing pipeline
- [main/main.c](main/main.c) — FreeRTOS task, button logic, BLE connection handler
- [main/gatt_svr.c](main/gatt_svr.c) — BLE GATT server registration and notify
- [main/IMU.c](main/IMU.c) — ICM-20948 I2C driver
- [bluetooth_test.py](bluetooth_test.py) — Python host application

### B.1 Key Tunable Constants (drum_pipeline.h)

```c
#define DP_GYRO_SCALE       131.0f   // LSB per deg/s (±250 dps range)
#define DP_ACCEL_SCALE      16384.0f // LSB per g (±2g range)
#define DP_ALPHA            0.95f    // complementary filter weight (pitch)
#define DP_MOVING_GYRO_DPS  8.0f     // threshold for "stick is moving"
#define DP_ACCEL_TRUST_G    0.3f     // max accel deviation to trust for pitch
#define DP_IMPACT_FREEZE_MS 40       // freeze pitch correction after hit (ms)
#define DP_HIT_THRESHOLD_G  2.0f     // minimum peak-g to register a hit
#define DP_DOWNSWING_GX_MIN 30.0f    // minimum gx rate for valid downswing (dps)
#define DP_HIT_COOLDOWN_MS  200      // minimum time between hits (ms)
#define DP_RING_SIZE        16       // IMU sample ring buffer depth
#define DP_PRE_HIT_SAMPLES  6        // samples before hit to use for pre_roll (60ms)
```

### B.2 Key Python Constants (bluetooth_test.py)

```python
SYNC_S      = 0.009   # 9ms sync window for two-stick simultaneity (requires timeBeginPeriod(1))
ZONE_LO     = -30.0   # snare/R-TOM boundary (degrees)
ZONE_HI     = +30.0   # R-TOM/floor tom boundary (degrees)
VOL_MIN_G   = 2.0     # g value at which volume = VOL_FLOOR
VOL_MAX_G   = 10.0    # g value at which volume = 100%
VOL_FLOOR   = 0.15    # minimum volume (softest possible hit)
```

### B.3 IMU Configuration Register Values (IMU.c)

```c
// Bank 2 — written during imu_init()
// 0x39 = (DLPFCFG=7 << 3) | (FS_SEL=0 << 1) | FCHOICE=1
imu_write_reg(REG_GYRO_CONFIG_1, 0x39);  // 361 Hz BW, 0.23 ms group delay, ±250 dps
imu_write_reg(REG_ACCEL_CONFIG,  0x39);  // 473 Hz BW, 1.0 ms group delay,  ±2g
```

### B.4 FreeRTOS Tick Rate (sdkconfig)

```
CONFIG_FREERTOS_HZ=1000   # 1ms tick — reduces vTaskDelay jitter from ±10ms to ±1ms
```

---

## Appendix C — Senior Project Analysis Form

**Project Title:** Wireless IMU-Based Drumstick Controllers

**Quarter / Year Submitted:** Spring / June 2026

**Student:** Tyler Bodenhamer

---

**Summary of Functional Requirements:**

The system allows a user to play electronic drums in open air using two wireless sticks. Each stick detects downstrokes using an inertial measurement unit, classifies the hit by wrist heading angle into one of three drum zones, and transmits the result to a host PC over Bluetooth Low Energy. The PC plays the corresponding sampled drum or cymbal sound with velocity-sensitive volume. A wrist button on each stick provides heading recalibration (short press) and cymbal mode (hold).

---

**Primary Constraints:**

The most significant constraint was the BLE radio architecture of the ESP32-C3, which physically cannot transmit notifications from two connections at the same instant. This required a software synchronization layer in the host that would not have been needed with a wired connection or Wi-Fi. A secondary constraint was the Windows default timer resolution of 15.6 ms, which made millisecond-precision timing in Python impossible without explicitly requesting a higher resolution via the Windows Multimedia API. A third constraint was the ±2g accelerometer range, which saturates during hard strikes and limits the accuracy of single-sample peak measurement.

---

**Economic:**

- Original estimated component cost: $30 (two sticks)
- Actual final component cost: $18.90 (two sticks, see Appendix A)
- Additional development equipment: ~$11
- Original estimated development time: 6 weeks
- Actual development time: 10 weeks (BLE timing problems took longer than expected)

---

**Environmental:**

The sticks contain no hazardous materials beyond the small amounts of lead-free solder used in assembly. The ESP32-C3 and ICM-20948 contain RoHS-compliant components. Battery disposal would be relevant if a LiPo were added in a future revision.

---

**Manufacturability:**

Current construction requires hand-soldering of fine-pitch breakout boards and routing of 28 AWG hookup wire inside a drumstick. Scaling to production would require a custom single-board design with the ESP32-C3 module and ICM-20948 integrated on one PCB, reducing assembly to a single solder operation.

---

**Sustainability:**

The system draws power from a USB-C port (either a laptop or USB battery). Power consumption is approximately 80–120 mA at 3.3V during active BLE operation. A 1000 mAh LiPo battery would provide 8–12 hours of use. No special maintenance is required beyond firmware updates, which are flashed over USB.

Upgrades that would improve the design: custom PCB, LiPo battery with USB charging, ±8g accelerometer range, MIDI output.

---

**Ethical:**

The system produces sound through a laptop speaker or headphones. There are no privacy concerns. The BLE characteristic UUID is custom (not a standard service) and the device does not transmit any identifying information beyond IMU data. No data is stored or transmitted to any server.

---

**Health and Safety:**

The sticks are held and swung by the user. At playing speeds, the tip of the stick moves at approximately 5–10 m/s. No modification is made to the mechanical structure of the drumstick; the electronics are surface-mounted or embedded without altering the stick's structural integrity. Eye protection is recommended during the soldering phase of construction.

---

**Social and Political:**

This project has potential social benefit for musicians who live in noise-restricted environments (apartments, dormitories, shared housing) by enabling silent or near-silent drum practice. The complete open-source implementation makes it accessible to anyone with the hardware components and a laptop.

---

**Development:**

New tools and techniques learned during this project:

- **ESP-IDF framework and FreeRTOS** — prior experience was with Arduino; this project required learning component-level CMake build configuration, FreeRTOS task creation, and NimBLE GATT server registration.
- **BLE GATT protocol** — understanding of characteristic handles, notification subscriptions, connection parameter negotiation, and the timing implications of connection intervals.
- **Python asyncio** — the cooperative concurrency model, the distinction between async and threading-based parallelism, and the specific behavior of bleak's BLE callbacks within an asyncio event loop.
- **Windows multimedia APIs** — `timeBeginPeriod` / `timeEndPeriod` for timer resolution, and `CoInitializeEx` for COM apartment model configuration required by Windows BLE.
- **Complementary filter design** — deriving the alpha coefficient from desired cutoff frequency and sample rate, and the specific cases in which to disable the accelerometer correction term.
- **IMU bias estimation** — exponential moving average bias learning for gyroscope zero-offset, and the conditions (stationary, low acceleration noise, not post-impact) under which it is safe to update the estimate.
