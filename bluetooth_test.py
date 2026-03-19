import asyncio
import os
from bleak import BleakClient

ADDRESS = "1c:db:d4:33:58:a2"
CHAR_UUID = "33333333-2222-2222-1111-111100000000"

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

SFX = {}

def init_audio():
    import pygame
    pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=256)
    SFX[("Up", "Left")]    = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "left_tom.wav"))
    SFX[("Up", "Right")]   = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "right_tom.wav"))
    SFX[("Down", "Left")]  = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "snare.wav"))
    SFX[("Down", "Right")] = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "floor_tom.wav"))

PITCH_THRESHOLD = -60  # below = up (toms), above = down (snare/floor)
hit_counter = 0

def classify_hit(pitch, roll):
    global hit_counter
    vertical = "Up" if pitch < PITCH_THRESHOLD else "Down"
    horizontal = "Left" if roll < 0 else "Right"
    hit_counter += 1
    return vertical, horizontal

# simple bar display for orientation
def angle_bar(val, label):
    clamped = max(-90, min(90, val))
    pos = int(clamped / 90 * 20)
    left = '\u2588' * abs(min(pos, 0))
    right = '\u2588' * max(pos, 0)
    return f"  {label} {left:>20}|{right:<20} {val:+7.1f}\u00b0"

def accel_bar(val, label):
    clamped = max(0, min(5, val))
    filled = int(clamped / 5 * 40)
    return f"  {label} {'▓' * filled:<40} {val:5.2f}g"

def callback(sender, data):
    try:
        text = data.split(b'\x00')[0].decode()
        parts = text.split(',')

        if parts[0] == 'S' and len(parts) == 4:
            pitch = float(parts[1])
            roll = float(parts[2])
            amag = float(parts[3])
            print(f"\033[H\033[J"
                  f"{angle_bar(pitch, 'Pitch')}\n"
                  f"{angle_bar(roll, 'Roll ')}\n"
                  f"{accel_bar(amag, 'Accel')}", flush=True)

        elif parts[0] == 'H' and len(parts) == 10:
            pitch  = float(parts[1])
            roll   = float(parts[2])
            peak_g = float(parts[7])
            ax     = float(parts[8])
            ay     = float(parts[9])

            vertical, horizontal = classify_hit(pitch, roll)
            print(f"HIT #{hit_counter}  {vertical} {horizontal}  "
                  f"P:{pitch:+.1f} ax:{ax:+.3f} ay:{ay:+.3f}  "
                  f"peak:{peak_g:.1f}g", flush=True)
            sound = SFX.get((vertical, horizontal))
            if sound:
                sound.play()

    except Exception:
        pass

async def main():
    async with BleakClient(ADDRESS) as client:
        print("Connected. Receiving IMU data...\n")
        init_audio()
        await client.start_notify(CHAR_UUID, callback)
        await asyncio.sleep(999)

asyncio.run(main())
