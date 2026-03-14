import asyncio
import math
import os
from collections import deque
from bleak import BleakClient
ADDRESS = "1c:db:d4:33:58:a2"
CHAR_UUID = "33333333-2222-2222-1111-111100000000"

WINDOW_SIZE = 50
DT = 0.05
GYRO_SCALE = 131.0
ACCEL_SCALE = 16384.0
SMOOTH_WINDOW = 5

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Read wav files as raw bytes at import time (no audio device init)
import wave
import numpy as np

def _load(name):
    path = os.path.join(SCRIPT_DIR, name)
    with wave.open(path, 'rb') as w:
        sr = w.getframerate()
        frames = w.readframes(w.getnframes())
        data = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
        if w.getnchannels() > 1:
            data = data.reshape(-1, w.getnchannels())
    return data, sr

SFX = {
    ("Up", "Left"):    _load("left_tom.wav"),
    ("Up", "Right"):   _load("right_tom.wav"),
    ("Down", "Left"):  _load("snare.wav"),
    ("Down", "Right"): _load("floor_tom.wav"),
}

_sd = None

def init_audio():
    """Call after bleak connects to avoid COM threading conflict."""
    global _sd
    import sounddevice as sd
    _sd = sd

def play_sound(key):
    if _sd is None:
        return
    data, sr = SFX[key]
    _sd.play(data, sr)

window = deque(maxlen=WINDOW_SIZE)
hit_status = False
debounce_counter = 0
hit_counter = 0
angle_x = 0.0
angle_y = 0.0
angle_z = 0.0

# Track position at last hit for relative movement detection
last_hit_x = 0.0
last_hit_y = 0.0
last_vertical = "Down"
last_horizontal = "Left"
MOVE_THRESHOLD = 20  # degrees of change needed to count as movement on an axis

def weighted_recent_average(samples, n=SMOOTH_WINDOW):
    if not samples:
        return None

    recent = list(samples)[-min(n, len(samples)):]  # oldest -> newest
    weights = list(range(1, len(recent) + 1))       # newest gets highest weight
    total_w = sum(weights)

    sums = [0.0] * 6
    for w, row in zip(weights, recent):
        for i, v in enumerate(row):
            sums[i] += w * v

    return [v / total_w for v in sums]

def process(sample):
    global hit_status, debounce_counter, hit_counter
    global angle_x, angle_y, angle_z

    smoothed = weighted_recent_average(window, 1)
    if smoothed is None:
        return

    ax, ay, az, gx, gy, gz = smoothed

    DEAD_ZONE = 1  # ignore changes smaller than this (degrees)
    dx = (gx / GYRO_SCALE) * DT
    dy = (gy / GYRO_SCALE) * DT
    dz = (gz / GYRO_SCALE) * DT
    if abs(dx) >= DEAD_ZONE:
        angle_x += dx
    if abs(dy) >= DEAD_ZONE:
        angle_y += dy
    if abs(dz) >= DEAD_ZONE:
        angle_z += dz

    BAR_WIDTH = 20  # half-width in characters

    def angle_bar(val, label):
        clamped = max(-90, min(90, val))
        pos = int(clamped / 90 * BAR_WIDTH)
        left = '█' * abs(min(pos, 0))
        right = '█' * max(pos, 0)
        return f"  {label} {left:>{BAR_WIDTH}}|{right:<{BAR_WIDTH}} {val:+7.1f}°"

    def accel_bar(val, label):
        clamped = max(-32768, min(32767, val))
        pos = int(clamped / 32768 * BAR_WIDTH)
        left = '▓' * abs(min(pos, 0))
        right = '▓' * max(pos, 0)
        return f"  {label} {left:>{BAR_WIDTH}}|{right:<{BAR_WIDTH}} {val:+7.0f}"

    lines = (
        f"{angle_bar(angle_x, 'AngX')}\n"
        f"{angle_bar(angle_y, 'AngY')}\n"
        f"{angle_bar(angle_z, 'AngZ')}\n"
        f"{accel_bar(ax, 'AccX')}\n"
        f"{accel_bar(ay, 'AccY')}\n"
        f"{accel_bar(az, 'AccZ')}"
    )
    os.system('cls')
    print(lines, flush=True)
    
    if az > 30000 and gx>0 and debounce_counter > 10:
        hit_status = True
            
    # only run when acceleration just finished
    elif hit_status:
        global last_hit_x, last_hit_y, last_vertical, last_horizontal

        delta_x = angle_x - last_hit_x
        delta_y = angle_y - last_hit_y

        # Only update an axis if the change is significant, otherwise keep last
        # delta_x decreased → moved from Down to Up, increased → Up to Down
        if abs(delta_x) >= MOVE_THRESHOLD:
            vertical = "Up" if delta_x < 0 else "Down"
        else:
            vertical = last_vertical

        # delta_y decreased → moved from Right to Left, increased → Left to Right
        if abs(delta_y) >= MOVE_THRESHOLD:
            horizontal = "Left" if delta_y < 0 else "Right"
        else:
            horizontal = last_horizontal

        last_hit_x = angle_x
        last_hit_y = angle_y
        last_vertical = vertical
        last_horizontal = horizontal
        angle_x = 0.0
        angle_y = 0.0   
        angle_z = 0.0
        debounce_counter = 0
        hit_counter += 1
        print("Hit",vertical,horizontal,flush=True)
        play_sound((vertical, horizontal))
        hit_status = False
    else:
        debounce_counter += 1
        

def callback(sender, data):
    try:
        text = data.split(b'\x00')[0].decode()
        vals = [int(v) for v in text.split(',')]
        if len(vals) == 6:
            window.append(vals)
            process(vals)
    except Exception:
        pass

async def reset_angles():
    global angle_x, angle_y, angle_z
    angle_x = 0.0
    angle_y = 0.0
    angle_z = 0.0

async def main():
    async with BleakClient(ADDRESS) as client:
        print("Connected. Receiving IMU data...\n")
        init_audio()
        await client.start_notify(CHAR_UUID, callback)
        # asyncio.create_task(reset_angles())
        await asyncio.sleep(999)

asyncio.run(main())
