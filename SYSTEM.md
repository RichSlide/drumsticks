# BLE Drum Sticks — Complete System Reference

Two wireless drumstick controllers, each built from an ESP32-C3 SuperMini and an ICM-20948 9-axis IMU, stream motion data over Bluetooth Low Energy to a Windows Python host that maps stick orientation and swing force to drum or cymbal sounds in real time.

---

## Table of Contents

1. [Hardware](#1-hardware)
2. [Repository Layout](#2-repository-layout)
3. [Firmware Architecture](#3-firmware-architecture)
   - [IMU Driver (IMU.c / IMU.h)](#31-imu-driver)
   - [Signal Processing Pipeline (drum_pipeline.c / drum_pipeline.h)](#32-signal-processing-pipeline)
   - [GATT Server (gatt_svr.c / bleprph.h)](#33-gatt-server)
   - [Application Entry Point (main.c)](#34-application-entry-point)
   - [Build System (CMakeLists.txt, sdkconfig)](#35-build-system)
4. [BLE Protocol](#4-ble-protocol)
5. [Python Host (bluetooth_test.py)](#5-python-host)
   - [Connection Management](#51-connection-management)
   - [Packet Parsing & State Machine](#52-packet-parsing--state-machine)
   - [Zone Classification](#53-zone-classification)
   - [Audio Playback](#54-audio-playback)
   - [Terminal Visualisation](#55-terminal-visualisation)
6. [End-to-End Data Flow](#6-end-to-end-data-flow)
7. [Signal Processing Deep Dive](#7-signal-processing-deep-dive)
8. [Button Behaviour](#8-button-behaviour)
9. [Tunable Parameters Reference](#9-tunable-parameters-reference)
10. [Building and Flashing](#10-building-and-flashing)
11. [Running the Python Host](#11-running-the-python-host)
12. [Calibration and Tuning Guide](#12-calibration-and-tuning-guide)

---

## 1. Hardware

### Per-Stick Bill of Materials

| Component | Part | Notes |
|---|---|---|
| Microcontroller | ESP32-C3 SuperMini | RISC-V, 4 MB flash, integrated 2.4 GHz radio |
| IMU | InvenSense ICM-20948 | 3-axis accel + 3-axis gyro + 3-axis mag (mag unused) |
| Button | Momentary tactile switch | Shorts BTN_DRIVE_GPIO to BTN_READ_GPIO |
| Power | USB-C or 3.7 V LiPo + regulator | Not specified in firmware |

### Wiring

```
ICM-20948          ESP32-C3 SuperMini
─────────          ──────────────────
VCC  ──────────►  3V3
GND  ──────────►  GND
SDA  ──────────►  GPIO 6
SCL  ──────────►  GPIO 5
AD0  ──────────►  GND   (I2C address = 0x68)

Button
──────
One pin ──────────►  GPIO 1   (BTN_DRIVE_GPIO, driven HIGH)
Other pin ────────►  GPIO 3   (BTN_READ_GPIO, internal pull-down)
```

### ICM-20948 Configuration

The IMU is configured in Bank 2 at startup:

| Register | Value | Meaning |
|---|---|---|
| GYRO_CONFIG_1 (0x01) | 0x01 | ±500 dps full-scale |
| ACCEL_CONFIG (0x14) | 0x01 | ±4 g full-scale |

> **Note:** The firmware constants `DP_GYRO_SCALE = 131.0` and `DP_ACCEL_SCALE = 16384.0` match the ICM-20948 default ±250 dps / ±2 g sensitivity values (the register config above upgrades range but the scale constants were not updated — if readings seem half-range, this is why).

### Axis Mapping (as-mounted)

```
gx = pitch rate   — positive = downswing (stick tip moving down)
gy = wrist-twist rate
gz = yaw rate     — positive = pointing left, negative = pointing right
ax, ay, az = linear acceleration components
```

---

## 2. Repository Layout

```
bleprph/
│
├── main/                        ← ESP-IDF component (firmware)
│   ├── CMakeLists.txt           ← Component source list and IDF deps
│   ├── Kconfig.projbuild        ← menuconfig options (BLE security, etc.)
│   ├── idf_component.yml        ← References nimble_peripheral_utils
│   │
│   ├── IMU.h                    ← I2C driver public API
│   ├── IMU.c                    ← ICM-20948 init + raw register reads
│   │
│   ├── drum_pipeline.h          ← All tunable constants + struct definitions
│   ├── drum_pipeline.c          ← Orientation filter, hit detection logic
│   │
│   ├── bleprph.h                ← GATT UUID constants + gatt_svr declarations
│   ├── gatt_svr.c               ← NimBLE GATT service definition + notify helper
│   │
│   └── main.c                   ← app_main, BLE stack init, imu_task, button logic
│
├── CMakeLists.txt               ← Top-level IDF project (sets project name, mbedTLS preset)
├── sdkconfig                    ← Generated KConfig (do not hand-edit)
├── sdkconfig.defaults           ← BT_NIMBLE_ENABLED, BLE-only mode
├── sdkconfig.defaults.mini      ← Stripped-down profile for SuperMini flash size
│
├── bluetooth_test.py            ← Python BLE host — connects, classifies, plays sounds
├── generate_sounds.py           ← Utility: generates placeholder .wav tones (unused in prod)
│
├── snare.wav                    ← Drum sounds loaded by bluetooth_test.py
├── tom.wav
├── floor_tom.wav
├── hihat.wav
├── crash.wav
├── ride.wav
│
├── .venv/                       ← Python virtual environment (bleak + pygame)
├── .vscode/                     ← VS Code launch configs + IntelliSense (clangd)
├── .devcontainer/               ← Optional Docker dev container
├── build/                       ← CMake / IDF build artefacts (gitignored)
└── tutorial/                    ← Original bleprph walkthrough from Espressif
```

---

## 3. Firmware Architecture

### 3.1 IMU Driver

**Files:** [main/IMU.h](main/IMU.h), [main/IMU.c](main/IMU.c)

The IMU driver is a thin I2C HAL that owns one `i2c_master_bus_handle_t` and one `i2c_master_dev_handle_t` as file-scoped statics.

#### `imu_init()`

1. Creates an I2C master bus on SDA=GPIO6, SCL=GPIO5 at 400 kHz with internal pull-ups enabled.
2. Adds the ICM-20948 at address `0x68` (AD0 tied low).
3. Waits 50 ms for power-on reset.
4. Reads `WHO_AM_I` register (Bank 0, addr `0x00`) — expects `0xEA`. Failure triggers an I2C bus scan and returns `ESP_FAIL`.
5. Sets `PWR_MGMT_1 = 0x01` (wake, auto-select clock) and `PWR_MGMT_2 = 0x00` (all axes enabled).
6. Switches to Bank 2, writes gyro and accel config registers, switches back to Bank 0.

#### `imu_read_accel_gyro_raw()`

Issues one I2C transmit-receive transaction: sends register address `0x2D` (ACCEL_XOUT_H), receives 12 bytes covering all 6 axes in big-endian signed 16-bit format. Assembles into `int16_t` ax/ay/az/gx/gy/gz.

Called every 10 ms from `imu_task` in `main.c`.

#### `imu_i2c_scan()`

Probes all 7-bit addresses 0x03–0x77. Used only for diagnostics when `WHO_AM_I` fails.

---

### 3.2 Signal Processing Pipeline

**Files:** [main/drum_pipeline.h](main/drum_pipeline.h), [main/drum_pipeline.c](main/drum_pipeline.c)

This is the core of the firmware. All logic lives in two functions: `dp_init` and `dp_update`.

#### Data Structures

```c
// One IMU sample with orientation already computed
typedef struct {
    int64_t  timestamp_us;
    float    ax, ay, az;       // acceleration in g
    float    gx, gy, gz;       // angular rate in deg/s
    float    pitch, roll;      // orientation angles in degrees
    float    accel_mag;        // |a| in g
} dp_sample_t;

// Emitted when a hit is detected
typedef struct {
    int64_t  timestamp_us;
    float    pitch, roll;          // orientation at hit moment
    float    pre_pitch, pre_roll;  // orientation 3 samples before hit
    float    gyro_pitch_rate;      // gx at hit moment
    float    gyro_roll_rate;       // gz at hit moment
    float    accel_peak_g;         // max |a| in window around hit
    float    ax, ay;               // accel from pre-hit sample
} dp_hit_event_t;

// Persistent state — one per stick
typedef struct {
    float    pitch, roll;          // current angles
    float    prev_accel_mag;       // for rising-edge detection
    float    gyro_bias_z;          // learned gz DC offset
    int64_t  last_hit_us;          // timestamp of last accepted hit
    int64_t  freeze_until_us;      // accel correction disabled until
    bool     initialized;
    dp_sample_t ring[DP_RING_SIZE]; // circular buffer of recent samples
    int      ring_head;
    int      ring_count;
} dp_state_t;
```

#### `dp_init(dp_state_t *s)`

`memset` to zero, sets `initialized = false`. Everything else is set on the first call to `dp_update`.

#### `dp_update()` — Full Walkthrough

Called every ~10 ms with raw `int16_t` sensor data and the current `esp_timer_get_time()` timestamp.

**Step 1 — Unit conversion**
```c
ax = raw_ax / 16384.0f;   // → g
gz = raw_gz / 131.0f;     // → deg/s
```

**Step 2 — First-sample initialisation**

On the very first call, the pipeline:
- Initialises pitch from accelerometer: `pitch = atan2(ay, az)` in degrees.
- Initialises heading (roll) to **0.0** — the direction the stick points at startup is defined as "centre" (0°).
- Seeds `last_hit_us` to `now - cooldown` so the first hit can register immediately.

**Step 3 — dt calculation**

Computes elapsed time since the previous ring-buffer sample. Clamped to 100 ms to prevent integrator blow-up on first sample or after pauses.

**Step 4 — Pitch: complementary filter**

```
alpha = 1.0  if (|accel_mag - 1g| > 0.3g) or (freeze active)
alpha = 0.95 otherwise

pitch = alpha * (pitch + gx * dt) + (1 - alpha) * accel_pitch
```

- When the stick is at rest (accel magnitude close to 1 g, no freeze), accel provides 5% correction each sample to prevent gyro drift.
- During a hit (`freeze_until_us` active for 40 ms) or during high-g motion (which would corrupt the accel reference), the filter is pure gyro integration.
- The "moving" flag (`gyro_total > 8 dps`) is computed but used only to gate bias learning, not the filter alpha — the accel-magnitude check alone guards pitch.

**Step 5 — Heading (stored in `roll`): pure gz integration**

Unlike pitch, there is no accelerometer reference for yaw. The heading is pure integration:

```c
// Bias estimation — only while stationary and unfrozen
if (!moving && !noisy_accel && !frozen)
    gyro_bias_z = 0.995 * gyro_bias_z + 0.005 * gz;

roll = roll - (gz - gyro_bias_z) * dt;
```

The exponential moving average on `gyro_bias_z` converges in ~2 s of stillness. Removing the DC bias prevents the heading from drifting at several degrees per second while the stick is lying on a table. With 60° zones, typical sessions are fine without a magnetometer.

**Heading sign convention:** the subtraction (`roll - gz*dt`) means rotating the stick rightward (increasing gz) moves the heading left (more negative). If the cursor moves in the wrong direction, negate the gz term.

**Step 6 — Ring buffer**

The current sample (with computed pitch and roll) is written into a 16-entry circular buffer (`ring_head` advances, wraps at `DP_RING_SIZE=16`). `ring_ago(s, n)` retrieves the sample n positions back from the most recent.

**Step 7 — Hit detection**

Four conditions must all be true simultaneously:

```
1. accel_mag >= DP_HIT_THRESHOLD_G     (2.0 g)   — magnitude threshold
2. accel_mag > prev_accel_mag                     — rising edge (not a plateau)
3. ring_ago(1)->gx > DP_DOWNSWING_GX_MIN (30 dps) — downswing was happening 10 ms ago
4. now - last_hit_us >= DP_HIT_COOLDOWN_MS (200 ms) — debounce
```

On a hit:
- `last_hit_us` is updated.
- `freeze_until_us` is set to `now + 40 ms` — prevents vibration from corrupting the orientation filter.
- The `dp_hit_event_t` struct is populated:
  - `roll` and `pitch` from the current sample.
  - `pre_roll` and `pre_pitch` from `ring_ago(DP_PRE_HIT_SAMPLES)` = 3 samples back (~30 ms) — this is the arm's resting orientation before the swing disturbed it.
  - `accel_peak_g` is the maximum `accel_mag` across the current sample and up to `DP_PRE_HIT_SAMPLES + 2` = 5 samples back, to catch the true peak regardless of exactly which sample triggered.
  - `ax` / `ay` from the pre-hit sample — captures accelerometer values before inertial contamination from the impact.

#### Constants Summary

| Constant | Value | Purpose |
|---|---|---|
| `DP_GYRO_SCALE` | 131.0 | raw→deg/s divisor |
| `DP_ACCEL_SCALE` | 16384.0 | raw→g divisor |
| `DP_ALPHA` | 0.95 | complementary filter gyro weight for pitch |
| `DP_MOVING_GYRO_DPS` | 8.0 | threshold for "stick is moving" (gates bias learning) |
| `DP_ACCEL_TRUST_G` | 0.3 | max deviation from 1 g before accel is considered noisy |
| `DP_IMPACT_FREEZE_MS` | 40 | ms to disable accel correction after a hit |
| `DP_HIT_THRESHOLD_G` | 2.0 | minimum peak-g to register a hit |
| `DP_DOWNSWING_GX_MIN` | 30.0 | minimum pitch rate (deg/s) confirming a downswing |
| `DP_HIT_COOLDOWN_MS` | 200 | minimum ms between accepted hits (debounce) |
| `DP_RING_SIZE` | 16 | ring buffer capacity in samples |
| `DP_PRE_HIT_SAMPLES` | 3 | samples before hit used for pre-orientation |

---

### 3.3 GATT Server

**Files:** [main/gatt_svr.c](main/gatt_svr.c), [main/bleprph.h](main/bleprph.h)

The GATT server exposes a single custom primary service with one notifiable characteristic.

#### UUIDs

```
Service:        59 46 2F 12 95 43 99 99  12 C8 58 B4 59 A2 71 2D
Characteristic: 33 33 33 33 22 22 22 22  11 11 11 11 00 00 00 00
Descriptor:     34 34 34 34 23 23 23 23  12 12 12 12 01 01 01 01
```

The Python host subscribes to the characteristic UUID `"33333333-2222-2222-1111-111100000000"`.

#### Characteristic Flags

`BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE`

Encryption flag (`CONFIG_EXAMPLE_ENCRYPTION`) is disabled by default.

#### `gatt_svr_set_imu_payload_and_notify(data, len)`

The only function called from `imu_task`. It:
1. `memset`s the 64-byte static buffer to zero (ensures null-termination if len < 64).
2. `memcpy`s `data` into the buffer.
3. Calls `ble_gatts_chr_updated(handle)` which schedules a BLE notification to all subscribed peers.

The zero-padding means the Python host can safely call `data.split(b'\x00')[0]` to strip the null padding before decoding.

---

### 3.4 Application Entry Point

**File:** [main/main.c](main/main.c)

#### `app_main()`

Execution order:
1. `nvs_flash_init()` — required by BLE stack for PHY calibration storage.
2. `nimble_port_init()` — initialises the NimBLE host.
3. BLE host callbacks registered: `reset_cb`, `sync_cb`, `gatts_register_cb`, `store_status_cb`.
4. Security mode set to "just works" (no I/O capability, no bonding, no MITM).
5. `gatt_svr_init()` — registers the custom service with NimBLE.
6. `ble_svc_gap_device_name_set("nimble-bleprph")` — the advertised device name.
7. `nimble_port_freertos_init(bleprph_host_task)` — starts the NimBLE host task on FreeRTOS.
8. `scli_init()` — serial console (from nimble_peripheral_utils).
9. **`button_init()`** — configures GPIO 1 (output, HIGH) and GPIO 3 (input, pull-down).
10. `imu_init()` — sets up I2C and ICM-20948.
11. `xTaskCreate(imu_task, ...)` — starts the 10 ms IMU loop at priority 5.

#### `bleprph_on_sync()`

Called by NimBLE when the host is ready. Determines the public address type, prints the device address, and starts advertising with:
- Flags: general discoverable, BLE-only.
- TX power field: auto.
- Device name: `nimble-bleprph`.
- 16-bit UUID: `GATT_SVR_SVC_ALERT_UUID (0x1811)`.
- Connection mode: undirected connectable.
- Discovery mode: general.

#### `imu_task()`

Runs in an infinite loop with a 10 ms `vTaskDelay` at the bottom.

**Per-iteration logic:**

```
1. Read raw IMU data (12 bytes over I2C)
2. Read button GPIO level
3. Button state machine (see §8)
4. dp_update() → may emit hit event
5. If hit:    format and send H packet
   If no hit: format and send S packet
```

**S packet** (status, ~100 Hz):
```
"S,<pitch>,<roll>,<accel_mag>,<cymbal_mode>"
 e.g. "S,-3.2,14.7,0.98,0"
```

**H packet** (hit event):
```
"H,<pitch>,<roll>,<pre_pitch>,<pre_roll>,<gx_rate>,<gz_rate>,<peak_g>,<ax>,<ay>"
 e.g. "H,-5.1,22.3,-4.8,21.0,312.4,-8.2,7.45,0.021,-0.983"
```

Both are null-padded to 64 bytes by `gatt_svr_set_imu_payload_and_notify`.

---

### 3.5 Build System

**File:** [CMakeLists.txt](CMakeLists.txt) (top-level)

```cmake
cmake_minimum_required(VERSION 3.22)
list(APPEND sdkconfig_defaults
    C:/esp/v6.0-beta2/esp-idf/components/mbedtls/config/mbedtls_preset_bt.conf)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(bleprph)
```

`MINIMAL_BUILD` reduces compile time by excluding unused IDF components.

**File:** [main/CMakeLists.txt](main/CMakeLists.txt)

```cmake
idf_component_register(
    SRCS "drum_pipeline.c" "IMU.c" "main.c" "gatt_svr.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash bt console esp_driver_i2c esp_driver_gpio
)
```

**[sdkconfig.defaults](sdkconfig.defaults)**
```
CONFIG_BT_ENABLED=y
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BTDM_CTRL_MODE_BLE_ONLY=y
CONFIG_BT_BLUEDROID_ENABLED=n
```

Disables Bluedroid and BR/EDR to save ~200 kB flash.

**[main/Kconfig.projbuild](main/Kconfig.projbuild)**

Exposes `menuconfig` knobs:
- I/O capability (default: "just works" — no passkey required)
- Bonding (default: off)
- MITM security (default: off)
- Extended advertising (BLE 5.0, enabled if SOC supports it)
- Random address advertising
- Link encryption
- Peer address resolution

---

## 4. BLE Protocol

### Transport

- **Profile:** GATT over BLE 4.x (legacy advertising)
- **Advertising interval:** `BLE_GAP_ADV_FAST_INTERVAL1_MIN` (30 ms)
- **Connection interval:** negotiated by central (Python bleak), typically 7.5–30 ms
- **Notification:** every ~10 ms (matching IMU task rate), one packet per notification
- **MTU:** default 23 bytes (sufficient; packets are ≤ ~50 bytes ASCII + null)

### Packet Format

Both packet types are ASCII comma-separated values, null-padded to 64 bytes.

#### S Packet — Orientation Status

Sent every 10 ms when no hit is detected.

```
S,<pitch_deg>,<roll_deg>,<accel_mag_g>,<mode>
```

| Field | Type | Example | Description |
|---|---|---|---|
| `S` | literal | `S` | packet type identifier |
| pitch_deg | float %.1f | `-3.2` | current pitch angle, degrees |
| roll_deg | float %.1f | `14.7` | current heading (yaw), degrees from startup zero |
| accel_mag_g | float %.2f | `0.98` | \|acceleration\| in g |
| mode | int %d | `0` | `0` = drums, `1` = cymbals |

#### H Packet — Hit Event

Sent immediately when hit detection fires.

```
H,<pitch>,<roll>,<pre_pitch>,<pre_roll>,<gx_rate>,<gz_rate>,<peak_g>,<ax>,<ay>
```

| Field | Type | Example | Description |
|---|---|---|---|
| `H` | literal | `H` | packet type |
| pitch | float %.1f | `-5.1` | pitch at hit moment |
| roll | float %.1f | `22.3` | heading at hit moment |
| pre_pitch | float %.1f | `-4.8` | pitch 30 ms before hit |
| pre_roll | float %.1f | `21.0` | heading 30 ms before hit (used for zone classification) |
| gx_rate | float %.1f | `312.4` | pitch angular rate at hit (deg/s) |
| gz_rate | float %.1f | `-8.2` | yaw rate at hit (deg/s) |
| peak_g | float %.2f | `7.45` | max \|accel\| in the window around the hit |
| ax | float %.3f | `0.021` | pre-hit X acceleration (g) |
| ay | float %.3f | `-0.983` | pre-hit Y acceleration (g) — used to infer arm angle |

**Why `pre_roll` for classification?** By the time the impact registers (rising-edge of accel), the wrist has rotated toward the drum surface and `roll` has already moved from the true resting position. `pre_roll` captures where the stick was pointing *before* the swing.

---

## 5. Python Host

**File:** [bluetooth_test.py](bluetooth_test.py)

### Threading Model

Windows COM requires STA (Single-Threaded Apartment) for UI threads and MTA (Multi-Threaded Apartment) for background worker threads. Bleak's Windows backend uses WinRT which requires MTA. The standard Python asyncio `ProactorEventLoop` uses IOCP which ties to STA and causes COM threading errors with bleak.

The solution is to run everything inside a dedicated thread initialised as MTA with the `SelectorEventLoop`:

```python
def _run():
    ctypes.windll.ole32.CoInitializeEx(None, 0x0)           # MTA
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    asyncio.run(main())
    ctypes.windll.ole32.CoUninitialize()

t = threading.Thread(target=_run)
t.start()
t.join()
```

Everything — BLE connections, callbacks, audio playback, terminal rendering — runs in this single asyncio event loop. Since asyncio is cooperative and single-threaded, there are no mutex requirements for shared state like `sticks[]` or `recent_hits`.

---

### 5.1 Connection Management

```python
ADDRESSES = [
    "ac:a7:04:d3:2d:d6",  # stick 1
    "38:44:be:43:fc:de",  # stick 2
]
```

Each stick gets a dedicated `connect_stick(stick_idx)` coroutine. The two are launched concurrently with `asyncio.gather`.

**Connection stagger:** stick 1 waits `stick_idx * 12 = 12 s` before its first connection attempt. This prevents both sticks from hammering the Windows BLE stack simultaneously during initial connect, which can cause one to fail.

**Reconnection loop:** on any disconnect or error, the coroutine waits `3 + stick_idx * 2` seconds (3 s for stick 0, 5 s for stick 1) before retrying. The asymmetric delay again prevents simultaneous reconnect storms.

**Connection lifecycle:**
```
while True:
    s["status"] = "connecting…"  → render
    async with BleakClient(addr) as client:
        s["connected"] = True
        s["status"] = "ok"       → render
        await client.start_notify(CHAR_UUID, callback)
        while client.is_connected:
            await asyncio.sleep(0.5)   ← heartbeat check
    # BleakClient context manager handles disconnect
    s["connected"] = False       → render
    await asyncio.sleep(retry_delay)
```

---

### 5.2 Packet Parsing & State Machine

Each stick's callback is a closure capturing `stick_idx` and `s = sticks[stick_idx]`.

```python
def callback(_sender, data: bytes):
    text  = data.split(b'\x00')[0].decode()   # strip null padding
    parts = text.split(',')

    if parts[0] == 'S' and len(parts) == 5:
        # smooth heading, update accel display, update cymbal mode
        s["smoothed_roll"] = 0.7 * float(parts[2]) + 0.3 * s["smoothed_roll"]
        s["amag"]  = float(parts[3])
        s["mode"]  = int(parts[4])
        render()

    elif parts[0] == 'H' and len(parts) == 10:
        roll      = float(parts[4])        # pre_roll
        peak_g    = float(parts[7])
        zone      = classify(roll)         # SNARE / R-TOM / FLOOR TOM
        sound_key = CYMBAL_MAP[zone] if s["mode"] else zone
        s["last_hit"]      = zone
        s["smoothed_roll"] = roll
        recent_hits.append((hit_counter, stick_idx, sound_key, peak_g))
        render()
        sound = SFX.get(sound_key)
        if sound:
            ch = sound.play()
            if ch:
                ch.set_volume(hit_volume(peak_g))
```

**S packet smoothing:** `smoothed_roll = 0.7 * new + 0.3 * old` — this is an exponential moving average with α=0.7, giving a time constant of roughly 3 samples (~30 ms). It smooths cursor jitter without adding significant lag.

**H packet roll:** the Python host uses `parts[4]` (pre_roll) not `parts[2]` (roll at impact). This is intentional — see §4 above.

---

### 5.3 Zone Classification

```
ZONE_LO = -30°    ZONE_HI = +30°

roll < -30°  →  SNARE
-30° ≤ roll ≤ +30°  →  R-TOM
roll > +30°  →  FLOOR TOM
```

The 180° total range is divided into three 60° sectors centred on 0° (startup heading).

```python
def classify(roll):
    global hit_counter
    hit_counter += 1
    if roll < ZONE_LO:   return "SNARE"
    elif roll <= ZONE_HI: return "R-TOM"
    else:                 return "FLOOR TOM"
```

**Cymbal mode remapping:**
```python
CYMBAL_MAP = {"SNARE": "HIHAT", "R-TOM": "CRASH", "FLOOR TOM": "RIDE"}
```
When `s["mode"] == 1` (cymbal mode active), the sound key is remapped through this dict before audio playback.

---

### 5.4 Audio Playback

```python
SFX = {
    "SNARE":     pygame.mixer.Sound("snare.wav"),
    "R-TOM":     pygame.mixer.Sound("tom.wav"),
    "FLOOR TOM": pygame.mixer.Sound("floor_tom.wav"),
    "HIHAT":     pygame.mixer.Sound("hihat.wav"),
    "CRASH":     pygame.mixer.Sound("crash.wav"),
    "RIDE":      pygame.mixer.Sound("ride.wav"),
}
```

**Mixer initialisation:**
```python
pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=256)
pygame.mixer.set_num_channels(32)
```
- `buffer=256` samples at 44100 Hz = ~5.8 ms audio latency.
- 32 mixer channels prevent pool exhaustion — cymbal sounds can ring for 1–3 s, and with two sticks playing rapidly, the default 8 channels would be exhausted in seconds.

**Volume scaling:**

Hit force is mapped to volume on a linear scale:

```python
VOL_MIN_G = 2.0    # = DP_HIT_THRESHOLD_G — softest possible hit
VOL_MAX_G = 10.0   # hard hit → 100%
VOL_FLOOR = 0.15   # minimum volume so whisper hits are audible

def hit_volume(peak_g):
    t = (peak_g - VOL_MIN_G) / (VOL_MAX_G - VOL_MIN_G)
    return VOL_FLOOR + (1.0 - VOL_FLOOR) * max(0.0, min(1.0, t))
```

| peak_g | volume |
|---|---|
| 2.0 g (threshold) | 15% |
| 3.0 g | 26% |
| 5.0 g | 47% |
| 7.5 g | 72% |
| 10.0 g | 100% |

**Playback:**
```python
ch = sound.play()     # auto-selects a free channel from the 32-channel pool
if ch:                # ch is None only if all 32 channels are busy (very unlikely)
    ch.set_volume(hit_volume(peak_g))
```

---

### 5.5 Terminal Visualisation

The visualiser renders a "fan" shape in the terminal on every incoming packet, using ANSI escape codes to clear and redraw in place (`\033[H\033[J`).

#### Fan Geometry

```
  ╱──────────│────────────────────│──────────────╲
 ╱   SNARE   │      R-TOM         │   FLOOR TOM   ╲
╱            │         ◆          │                ╲
╱────────────┼────────────────────┼────────────────╲
-90°        -30°                 +30°             +90°
```

- `ARC_W = 48` columns — the horizontal span of the fan.
- `ARC_H = 5` rows — the vertical height.
- `ROLL_LO = -90°`, `ROLL_HI = +90°` — the displayed range.
- `B1 = _arc_col(-30°)`, `B2 = _arc_col(+30°)` — precomputed column indices for zone boundaries.

**Row layout:**
```
Row 0 — top edge (most indented, narrowest)
Row 1 — stick 1 cursor (◇)
Row 2 — zone labels (SNARE / R-TOM / FLOOR TOM)
Row 3 — stick 0 cursor (◆)
Row 4 — floor (────┼────┼────)
```

Sticks get dedicated rows so their cursors never overwrite each other, regardless of heading angles.

**Cursor position:** `_arc_col(smoothed_roll)` maps a heading angle to a column index:
```python
col = round((roll - ROLL_LO) / (ROLL_HI - ROLL_LO) * (ARC_W - 1))
```

**Below the fan:**
- Two status lines: one per stick, showing cursor symbol, roll angle, accel magnitude, current zone (highlighted green if it was the last-hit zone), and cymbal mode tag if active.
- Hit log: last 5 hits, showing hit counter, stick cursor, drum/cymbal name, and peak g.

---

## 6. End-to-End Data Flow

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  Physical World                                                              │
│                                                                              │
│  Drummer moves stick                                                         │
│         │                                                                    │
│         ▼                                                                    │
│  ICM-20948 samples ax/ay/az/gx/gy/gz at ~100 Hz (10 ms period)              │
│         │  I2C @ 400 kHz                                                     │
│         ▼                                                                    │
│  ESP32-C3 imu_task (every 10 ms via vTaskDelay)                              │
│         │  imu_read_accel_gyro_raw() → raw int16_t × 6                      │
│         ▼                                                                    │
│  dp_update()                                                                 │
│    ├── Convert raw → float (g, deg/s)                                        │
│    ├── Complementary filter → pitch (degrees)                                │
│    ├── Gyro bias learning → gyro_bias_z                                      │
│    ├── gz integration → roll/heading (degrees from startup zero)             │
│    ├── Write to ring buffer                                                  │
│    └── Hit detection (threshold + rising edge + downswing + cooldown)       │
│         │                                                                    │
│         ├── Hit detected ──► format H packet                                 │
│         └── No hit        ──► read button, format S packet                  │
│                                                                              │
│  gatt_svr_set_imu_payload_and_notify()                                       │
│    ├── memcpy into 64-byte static buffer                                     │
│    └── ble_gatts_chr_updated() → NimBLE schedules notification               │
│         │  BLE radio (~7–20 ms connection interval)                          │
│         ▼                                                                    │
│  Windows BLE stack (WinRT)                                                   │
│         │  bleak bridges to asyncio via call_soon_threadsafe                 │
│         ▼                                                                    │
│  Python asyncio event loop (WindowsSelectorEventLoopPolicy, MTA thread)     │
│         │                                                                    │
│         ▼                                                                    │
│  BLE callback (make_callback closure)                                        │
│    ├── Strip null padding, split CSV                                         │
│    ├── S packet: update smoothed_roll, amag, mode → render()                 │
│    └── H packet:                                                             │
│         ├── classify(pre_roll) → zone                                        │
│         ├── cymbal remap if mode==1                                          │
│         ├── append to recent_hits                                            │
│         ├── render()                                                         │
│         └── sound.play() → ch.set_volume(hit_volume(peak_g))                │
│                   │  SDL audio thread @ 44100 Hz, 256-sample buffer          │
│                   ▼                                                          │
│             Speakers / headphones                                            │
└─────────────────────────────────────────────────────────────────────────────┘
```

**Total latency (approximate):**
- IMU read: < 0.5 ms (12 bytes at 400 kHz I2C)
- dp_update: < 0.1 ms
- BLE notification: 7–30 ms (connection interval dependent)
- Python callback + sound.play(): < 1 ms
- Audio buffer: 256 / 44100 = ~5.8 ms
- **Total: ~15–40 ms** from impact to first audio sample

---

## 7. Signal Processing Deep Dive

### Why gz for heading (not tilt)?

The initial approach used `atan2(ax, sqrt(ay² + az²))` to measure the stick's L/R tilt. This is accurate for a static pose but breaks during arm movement: when you raise your arm to strike, gravity's projection onto the X axis changes even if you're pointing in the same direction. The result was the displayed zone shifting mid-swing, causing phantom zone changes.

The gz (yaw rate) approach measures **how fast the stick is rotating around the vertical axis** — this is exactly "which direction are you pointing." There is no absolute reference for yaw (gravity cannot tell you compass heading), so it's pure integration. The trade-off is drift, which is mitigated by bias estimation.

### Bias Estimation

The ICM-20948 gyro has a DC offset of typically 1–5 dps at rest. Without correction, a 3 dps bias integrates to 180° of drift per minute. The pipeline estimates this during stationary periods:

```
At each sample where:
  - gyro_total < 8 dps (stationary)
  - |accel_mag - 1g| < 0.3g (not during a hit)
  - not in freeze window

gyro_bias_z = 0.995 * gyro_bias_z + 0.005 * gz
```

This is a first-order IIR filter with τ ≈ 200 samples = 2 s. After about 6 s of stillness the bias has converged to within 1% of its true value, reducing drift to < 0.1 dps — which at 60° zones gives < 6° of drift per minute.

### Hit Detection Gate Analysis

The three-condition gate is designed to reject false positives:

| Condition | Rejects |
|---|---|
| `accel_mag >= 2.0 g` | Normal motion (< 1.5 g), gentle taps |
| `accel_mag > prev_accel_mag` | Second half of impact (bounce), plateau detection |
| `gx > 30 dps` (1 sample ago) | Upswings, lateral swings, wrist rotation without downstroke |
| `cooldown 200 ms` | Bounce re-triggers, stick rattle |

The 200 ms cooldown allows hitting up to 5 times per second, which is 16th notes at 75 BPM or 8th notes at 150 BPM. For faster playing, `DP_HIT_COOLDOWN_MS` can be reduced to 150 ms (risking some bounce triggers).

---

## 8. Button Behaviour

**Hardware:** BTN_DRIVE_GPIO (GPIO 1) is permanently pulled HIGH by the firmware. BTN_READ_GPIO (GPIO 3) has an internal pull-down and reads the button state.

**Press duration determines the action**, decided at release time:

```
Button pressed
    │
    ├── held < 500 ms ──► on release: dp_state.roll = 0.0f
    │                     (heading reset — the current direction becomes 0°)
    │
    └── held ≥ 500 ms ──► while held: S packet sends mode=1 (cymbal mode)
                          on release: mode returns to 0 (drums)
```

**State machine in `imu_task`:**

```c
static int     btn_prev       = 0;
static int64_t btn_press_us   = 0;
static bool    btn_long_fired = false;

// Each 10 ms iteration:
int btn_now = gpio_get_level(BTN_READ_GPIO);

if (btn_now && !btn_prev) {          // rising edge
    btn_press_us   = now_us;
    btn_long_fired = false;
}

bool cymbal_mode = false;
if (btn_now) {
    if (now_us - btn_press_us >= 500000LL) {
        cymbal_mode    = true;
        btn_long_fired = true;
    }
} else if (!btn_now && btn_prev) {   // falling edge
    if (!btn_long_fired)
        dp_state.roll = 0.0f;        // heading reset
}
btn_prev = btn_now;
```

The heading reset works because `dp_state.roll` is set to 0 before the next `dp_update` call in the same iteration. `dp_update` then continues integrating gz from 0, so the very next S packet carries a near-zero heading.

---

## 9. Tunable Parameters Reference

### Firmware (drum_pipeline.h)

| Constant | Current Value | Effect of Increasing | Effect of Decreasing |
|---|---|---|---|
| `DP_HIT_THRESHOLD_G` | 2.0 g | Fewer false triggers, misses gentle hits | More sensitive, may false-trigger |
| `DP_DOWNSWING_GX_MIN` | 30 dps | Requires more aggressive downswing | May trigger on lateral/upward swings |
| `DP_HIT_COOLDOWN_MS` | 200 ms | Slower max hit rate, better bounce rejection | Faster playing, risk of bounce triggers |
| `DP_IMPACT_FREEZE_MS` | 40 ms | More stable orientation after hit | Orientation may drift during vibration |
| `DP_ALPHA` | 0.95 | More gyro weight in pitch filter | More accel weight, jumpier pitch |
| `DP_ACCEL_TRUST_G` | 0.3 g | Wider range before accel is considered noisy | Stricter, accel rarely trusted |
| `DP_MOVING_GYRO_DPS` | 8.0 dps | Harder to enter "stationary" for bias learning | Bias update happens even during slow motion |
| `DP_PRE_HIT_SAMPLES` | 3 samples | Pre-orientation from further back (30 ms) | Pre-orientation from closer to impact |

### Python (bluetooth_test.py)

| Variable | Current Value | Effect |
|---|---|---|
| `ZONE_LO` | -30° | Left boundary of centre zone; move left to make snare zone wider |
| `ZONE_HI` | +30° | Right boundary of centre zone; move right to make floor tom zone wider |
| `VOL_MIN_G` | 2.0 g | Peak g that maps to minimum volume |
| `VOL_MAX_G` | 10.0 g | Peak g that maps to 100% volume |
| `VOL_FLOOR` | 0.15 | Volume for the softest detectable hit |
| S packet α | 0.7 | Smoothing for cursor display; 0.7 = fairly responsive |
| `ROLL_LO` / `ROLL_HI` | ±90° | Fan display angular range |

### Button Timing (main.c)

| Constant | Current Value | Effect |
|---|---|---|
| Long-press threshold | 500 ms (hardcoded) | Hold time before cymbal mode activates |

---

## 10. Building and Flashing

### Prerequisites

- ESP-IDF v6.0-beta2 or later installed at `C:\esp\v6.0-beta2\esp-idf`
- `IDF_PATH` environment variable set
- ESP32-C3 USB driver installed (CH340 or CP210x depending on SuperMini variant)

### Build

```powershell
# from repository root
idf.py set-target esp32c3
idf.py build
```

### Flash

```powershell
idf.py -p COM<N> flash
```

Hold the BOOT button on the SuperMini while pressing RESET if the device does not enter download mode automatically.

### Monitor

```powershell
idf.py -p COM<N> monitor
```

Expected startup output:
```
I (xxx) IMU: ICM-20948 initialized on SDA=6, SCL=5
I (xxx) NimBLE_BLE_PRPH: BLE Host Task Started
I (xxx) NimBLE_BLE_PRPH: Device Address: ac:a7:04:d3:2d:d6
```

If `WHO_AM_I mismatch` appears, check SDA/SCL wiring and that AD0 is tied low.

### Both Sticks

Each SuperMini has a unique MAC address. After flashing both sticks, update `ADDRESSES` in `bluetooth_test.py` with the addresses shown in the monitor output.

---

## 11. Running the Python Host

### Setup (first time)

```powershell
cd C:\Users\tyler\bleprph
python -m venv .venv
.venv\Scripts\Activate.ps1
pip install bleak pygame
```

### Run

```powershell
.venv\Scripts\Activate.ps1
python bluetooth_test.py
```

The terminal clears and shows the fan visualiser immediately. Status lines show "waiting" → "connecting…" → roll/accel data as sticks connect. Stick 1 starts connecting 12 s after stick 0.

### Sound Files

The following `.wav` files must be present in the repository root:

| File | Drum | Cymbal substitute |
|---|---|---|
| `snare.wav` | Snare drum | — |
| `tom.wav` | Right tom | — |
| `floor_tom.wav` | Floor tom | — |
| `hihat.wav` | — | Hi-hat (replaces snare in cymbal mode) |
| `crash.wav` | — | Crash (replaces R-tom) |
| `ride.wav` | — | Ride (replaces floor tom) |

`generate_sounds.py` generates simple sine-wave tones if real samples are unavailable:
```powershell
python generate_sounds.py
```
Note it generates `hit_up.wav` / `hit_down.wav` etc. (old naming), not the names above. Real drum samples are strongly recommended.

---

## 12. Calibration and Tuning Guide

### Cursor moves in wrong direction

The heading integration uses `roll = roll - gz * dt`. If pointing the stick right makes the cursor move left, negate gz:

In [main/drum_pipeline.c](main/drum_pipeline.c):
```c
s->roll = s->roll + (gz - s->gyro_bias_z) * dt;  // was minus
```

### Snare fires too often / floor tom too rarely

Shift `ZONE_LO` and `ZONE_HI` in [bluetooth_test.py](bluetooth_test.py):
```python
ZONE_LO = -40.0   # snare zone is now -90 to -40 (narrower)
ZONE_HI =  20.0   # floor tom zone is now +20 to +90 (wider)
```

Also use the heading reset (short-press button) to re-zero the heading when the stick is pointed at the snare — then the centre zone is correctly positioned.

### Heading drifts too fast

Reduce `DP_MOVING_GYRO_DPS` from 8 to 5 (allows bias learning during slower motion):

In [main/drum_pipeline.h](main/drum_pipeline.h):
```c
#define DP_MOVING_GYRO_DPS  5.0f
```

### Hits not registering on fast rolls

Lower `DP_HIT_COOLDOWN_MS`:
```c
#define DP_HIT_COOLDOWN_MS  150
```

Bounce re-triggers may appear on hard surfaces. If so, increase `DP_DOWNSWING_GX_MIN` slightly to compensate.

### Hit fires too early in the downswing

The hit fires on the rising edge of `accel_mag`, which is slightly before the true peak. This is by design for low latency. If timing feels too early, increase `DP_HIT_THRESHOLD_G` — this delays the trigger to a higher point on the rising edge:
```c
#define DP_HIT_THRESHOLD_G  2.5f
```

### Orientation jumps after hitting hard surface

Increase `DP_IMPACT_FREEZE_MS` — the accel correction is currently frozen for only 40 ms after impact:
```c
#define DP_IMPACT_FREEZE_MS  80
```
