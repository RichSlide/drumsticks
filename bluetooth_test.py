import asyncio
import os
from bleak import BleakClient
from collections import deque

CHAR_UUID  = "33333333-2222-2222-1111-111100000000"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── sticks ────────────────────────────────────────────────────────────────────
ADDRESSES = [
    "ac:a7:04:d3:2d:d6",  # stick 1
    "38:44:be:43:fc:de"  # stick 2  ← fill in
]
CURSORS = ["◆", "◇"]

sticks = [
    {
        "addr":          addr,
        "cursor":        CURSORS[i],
        "pitch":         0.0,
        "roll":          0.0,
        "amag":          0.0,
        "smoothed_roll": 0.0,
        "last_hit_zone": None,
        "connected":     False,
        "status":        "waiting",
    }
    for i, addr in enumerate(ADDRESSES)
]

# ── audio ─────────────────────────────────────────────────────────────────────
SFX = {}

def init_audio():
    import pygame
    pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=256)
    SFX[("Up",   "Left")]  = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "left_tom.wav"))
    SFX[("Up",   "Right")] = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "right_tom.wav"))
    SFX[("Down", "Left")]  = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "snare.wav"))
    SFX[("Down", "Right")] = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "floor_tom.wav"))

# ── classification ────────────────────────────────────────────────────────────
PITCH_THRESHOLD = -60    # below = Up (toms), above = Down (snare/floor)
ROLL_THRESHOLD  = -0.0  # negative bias: right wrist tends to read slightly left

DRUMS = {
    ("Up",   "Left"):  "LEFT TOM",
    ("Up",   "Right"): "RIGHT TOM",
    ("Down", "Left"):  "SNARE",
    ("Down", "Right"): "FLOOR TOM",
}

hit_counter  = 0
recent_hits  = deque(maxlen=7)

def classify(pitch, roll):
    global hit_counter
    hit_counter += 1
    return ("Up"   if pitch < PITCH_THRESHOLD else "Down",
            "Left" if roll  < ROLL_THRESHOLD  else "Right")

# ── grid ──────────────────────────────────────────────────────────────────────
GW, GH    = 40, 12
PITCH_LO  = -90
PITCH_HI  =   0
ROLL_LO   = -45.0   # degrees
ROLL_HI   =  45.0
PFX       =   7

def _gx(v): return max(0, min(GW-1, round((v - ROLL_LO)   / (ROLL_HI - ROLL_LO)       * (GW-1))))
def _gy(v): return max(0, min(GH-1, round((v - PITCH_LO)  / (PITCH_HI - PITCH_LO)     * (GH-1))))

TR = _gy(PITCH_THRESHOLD)
TC = _gx(ROLL_THRESHOLD)

def make_grid():
    cells = [[' '] * GW for _ in range(GH)]

    # zone boundary lines
    for c in range(GW): cells[TR][c] = '·'
    for r in range(GH): cells[r][TC] = '·'
    cells[TR][TC] = '+'

    # zone name labels
    for (v, h), (lr, lc) in {
        ("Up",   "Left"):  (TR // 2,        TC // 2),
        ("Up",   "Right"): (TR // 2,        (TC + GW) // 2),
        ("Down", "Left"):  ((TR + GH) // 2, TC // 2),
        ("Down", "Right"): ((TR + GH) // 2, (TC + GW) // 2),
    }.items():
        name = DRUMS[(v, h)]
        for i, ch in enumerate(name):
            c = lc - len(name) // 2 + i
            if 0 <= c < GW and cells[lr][c] == ' ':
                cells[lr][c] = ch

    # draw cursors for connected sticks
    for s in sticks:
        if not s["connected"]:
            continue
        cy = _gy(s["pitch"])
        cx = _gx(s["smoothed_roll"])
        cells[cy][cx] = s["cursor"]

    # assemble with pitch labels
    lines = [' ' * PFX + '┌' + '─' * GW + '┐']
    for r, row in enumerate(cells):
        if r == TR:
            lines.append(f"{PITCH_THRESHOLD:+{PFX-1}.0f}°├{'─'*TC}┼{'─'*(GW-TC-1)}┤")
        else:
            p = PITCH_LO + (PITCH_HI - PITCH_LO) * r / (GH - 1)
            lbl = f"{p:+{PFX-1}.0f}°" if r in (0, GH - 1) else ' ' * PFX
            lines.append(f"{lbl}│{''.join(row)}│")
    lines.append(' ' * PFX + '└' + '─' * TC + '┴' + '─' * (GW - TC - 1) + '┘')

    def place(buf, text, inner_col):
        start = PFX + 1 + inner_col - len(text) // 2
        for i, ch in enumerate(text):
            pos = start + i
            if 0 <= pos < len(buf):
                buf[pos] = ch

    roll_buf = [' '] * (PFX + 1 + GW + 2)
    place(roll_buf, f"{ROLL_LO:+.0f}°", 0)
    place(roll_buf, f"{ROLL_THRESHOLD:+.0f}°", TC)
    place(roll_buf, f"{ROLL_HI:+.0f}°", GW - 1)
    lines.append(''.join(roll_buf))

    return '\n'.join(lines)

# ── render ────────────────────────────────────────────────────────────────────
def render():
    BOLD  = "\033[1m"
    RESET = "\033[0m"
    YEL   = "\033[93m"
    GRN   = "\033[92m"
    DIM   = "\033[2m"
    CYN   = "\033[96m"

    # per-stick status line
    status_lines = []
    for s in sticks:
        if not s["connected"]:
            status_lines.append(
                f"  {CYN}{s['cursor']}{RESET} {DIM}{s['status']}{RESET}  {s['addr']}"
            )
            continue
        zone = ("Up"   if s["pitch"] < PITCH_THRESHOLD else "Down",
                "Left" if s["smoothed_roll"] < ROLL_THRESHOLD else "Right")
        drum = DRUMS[zone]
        is_last = (zone == s["last_hit_zone"])
        zone_str = f"{GRN}{BOLD}{drum}{RESET}" if is_last else f"{BOLD}{drum}{RESET}"
        status_lines.append(
            f"  {CYN}{s['cursor']}{RESET}  pitch:{s['pitch']:+6.1f}°  "
            f"roll:{s['smoothed_roll']:+.1f}°  accel:{s['amag']:.2f}g  ► {zone_str}"
        )

    log = []
    for n, stick_idx, z, peak in list(recent_hits)[::-1]:
        name = DRUMS.get(z, str(z))
        cursor = CURSORS[stick_idx]
        log.append(f"  {YEL}#{n:<3}{RESET} {CYN}{cursor}{RESET} {name:<12} {peak:.1f}g")
    while len(log) < 5:
        log.append(f"  {DIM}─{RESET}")

    total_str = f"total: {hit_counter}" if hit_counter else "none yet"
    print(
        "\033[H\033[J"
        + make_grid()
        + "\n\n"
        + "\n".join(status_lines)
        + f"\n\n  hits ({total_str})\n"
        + "\n".join(log),
        flush=True,
    )

# ── BLE callbacks ─────────────────────────────────────────────────────────────
def make_callback(stick_idx):
    s = sticks[stick_idx]

    def callback(_sender, data):
        try:
            text  = data.split(b'\x00')[0].decode()
            parts = text.split(',')

            if parts[0] == 'S' and len(parts) == 4:
                s["pitch"]        = float(parts[1])
                s["smoothed_roll"] = 0.7 * float(parts[2]) + 0.3 * s["smoothed_roll"]
                s["amag"]         = float(parts[3])
                render()

            elif parts[0] == 'H' and len(parts) == 10:
                pitch  = float(parts[1])
                peak_g = float(parts[7])
                roll   = float(parts[4])   # pre_roll: filtered roll before the swing
                v, h   = classify(pitch, roll)
                s["last_hit_zone"] = (v, h)
                recent_hits.append((hit_counter, stick_idx, s["last_hit_zone"], peak_g))
                s["pitch"]        = pitch
                s["smoothed_roll"] = roll
                render()
                sound = SFX.get(s["last_hit_zone"])
                if sound:
                    sound.play()

        except Exception:
            pass

    return callback

# ── connection ────────────────────────────────────────────────────────────────
async def connect_stick(stick_idx):
    s  = sticks[stick_idx]
    cb = make_callback(stick_idx)
    # Stagger starts so scans don't overlap (BLE scan timeout is 10 s)
    await asyncio.sleep(stick_idx * 12)
    while True:
        s["status"] = "connecting…"
        render()
        try:
            async with BleakClient(s["addr"]) as client:
                s["connected"] = True
                s["status"]    = "ok"
                render()
                await client.start_notify(CHAR_UUID, cb)
                while client.is_connected:
                    await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            s["status"] = str(e)[:60]      # show full message, not just type
        finally:
            s["connected"] = False
        render()
        await asyncio.sleep(3 + stick_idx * 2)   # stagger retries too

async def main():
    init_audio()
    render()
    await asyncio.gather(
        *(connect_stick(i) for i in range(len(ADDRESSES))),
        return_exceptions=True,
    )

import threading
import ctypes

def _run():
    # 1. Claim MTA before anything else touches COM in this thread.
    ctypes.windll.ole32.CoInitializeEx(None, 0x0)  # COINIT_MULTITHREADED
    # 2. Use SelectorEventLoop — avoids ProactorEventLoop's IOCP worker threads,
    #    which Windows initialises as STA and which WinRT BLE callbacks dispatch to.
    #    With SelectorEventLoop every callback runs on this single MTA thread.
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    try:
        asyncio.run(main())
    finally:
        ctypes.windll.ole32.CoUninitialize()

t = threading.Thread(target=_run)
t.start()
t.join()
