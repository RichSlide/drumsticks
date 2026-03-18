import asyncio
import os
from bleak import BleakClient

ADDRESS = "1c:db:d4:33:58:a2"
CHAR_UUID = "33333333-2222-2222-1111-111100000000"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# --- Audio setup (pygame.mixer with small buffer for low latency) ---

SFX = {}

def init_audio():
    """Initialise pygame.mixer after BLE connects to avoid COM conflict."""
    import pygame
    pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=256)
    SFX[("Up", "Left")]    = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "left_tom.wav"))
    SFX[("Up", "Right")]   = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "right_tom.wav"))
    SFX[("Down", "Left")]  = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "snare.wav"))
    SFX[("Down", "Right")] = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "floor_tom.wav"))

def play_sound(key):
    sound = SFX.get(key)
    if sound:
        sound.play()

# --- Relative drum position tracking ---

PITCH_THRESHOLD = -60  # degrees: below = Up (toms), above = Down (snare/floor)
AX_THRESHOLD = 0       # g: below = Left, above = Right
hit_counter = 0

def classify_hit(pitch, roll):
    """Determine which drum was hit using absolute pitch and accel X."""
    global hit_counter

    vertical = "Up" if pitch < PITCH_THRESHOLD else "Down"
    horizontal = "Left" if roll < 0 else "Right"

    hit_counter += 1
    return vertical, horizontal

# --- Visualiser ---

BAR_WIDTH = 20

def angle_bar(val, label):
    clamped = max(-90, min(90, val))
    pos = int(clamped / 90 * BAR_WIDTH)
    left = '\u2588' * abs(min(pos, 0))
    right = '\u2588' * max(pos, 0)
    return f"  {label} {left:>{BAR_WIDTH}}|{right:<{BAR_WIDTH}} {val:+7.1f}\u00b0"

def accel_bar(val, label):
    clamped = max(0, min(5, val))
    filled = int(clamped / 5 * BAR_WIDTH * 2)
    fill_char = '▓'
    return f"  {label} {fill_char * filled:<{BAR_WIDTH * 2}} {val:5.2f}g"

# --- BLE message handling ---
# Messages from ESP32 now come in two formats:
#   S,<pitch>,<roll>,<accel_mag>           — periodic state update
#   H,<pitch>,<roll>,<pre_p>,<pre_r>,<gp>,<gr>,<peak_g>  — hit event

def callback(sender, data):
    try:
        text = data.split(b'\x00')[0].decode()
        parts = text.split(',')
        msg_type = parts[0]

        if msg_type == 'S' and len(parts) == 4:
            # State update: show orientation
            pitch = float(parts[1])
            roll = float(parts[2])
            amag = float(parts[3])

            lines = (
                f"\033[H\033[J"
                f"{angle_bar(pitch, 'Pitch')}\n"
                f"{angle_bar(roll, 'Roll ')}\n"
                f"{accel_bar(amag, 'Accel')}"
            )
            print(lines, flush=True)

        elif msg_type == 'H' and len(parts) == 10:
            # Hit event: classify and play sound
            pitch     = float(parts[1])
            roll      = float(parts[2])
            pre_pitch = float(parts[3])
            pre_roll  = float(parts[4])
            gyro_p    = float(parts[5])
            gyro_r    = float(parts[6])
            peak_g    = float(parts[7])
            ax        = float(parts[8])
            ay        = float(parts[9])

            vertical, horizontal = classify_hit(pitch, roll)
            print(f"HIT #{hit_counter}  {vertical} {horizontal}  "
                  f"P:{pitch:+.1f} ax:{ax:+.3f} ay:{ay:+.3f}  "
                  f"peak:{peak_g:.1f}g",
                  flush=True)
            play_sound((vertical, horizontal))

    except Exception:
        pass

async def main():
    async with BleakClient(ADDRESS) as client:
        print("Connected. Receiving processed IMU data...\n")
        init_audio()
        await client.start_notify(CHAR_UUID, callback)
        await asyncio.sleep(999)

asyncio.run(main())
