import asyncio
import os
import threading
import ctypes
import time
import shutil
from bleak import BleakClient
from collections import deque

CHAR_UUID  = "33333333-2222-2222-1111-111100000000"
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── sticks ────────────────────────────────────────────────────────────────────
ADDRESSES = [
    "ac:a7:04:d3:2d:d6",  # stick 1
    "38:44:be:43:fc:de",  # stick 2
]
CURSORS = ["◆", "◇"]

sticks = [
    {
        "addr":          addr,
        "cursor":        CURSORS[i],
        "amag":          0.0,
        "smoothed_roll": 0.0,
        "last_hit":      None,
        "mode":          0,    # 0 = drums, 1 = cymbals
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
    pygame.mixer.set_num_channels(32)
    SFX["SNARE"]     = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "snare.wav"))
    SFX["R-TOM"]     = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "tom.wav"))
    SFX["FLOOR TOM"] = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "floor_tom.wav"))
    SFX["HIHAT"]     = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "hihat.wav"))
    SFX["CRASH"]     = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "crash.wav"))
    SFX["RIDE"]      = pygame.mixer.Sound(os.path.join(SCRIPT_DIR, "ride.wav"))

# drum zone → cymbal sound when mode=1
CYMBAL_MAP = {"SNARE": "HIHAT", "R-TOM": "CRASH", "FLOOR TOM": "RIDE"}

# volume scaling: hits at or below MIN_G get the floor volume; MAX_G and above = full
VOL_MIN_G   = 2.0   # matches DP_HIT_THRESHOLD_G — softest possible hit
VOL_MAX_G   = 10.0  # hard hit → 100%
VOL_FLOOR   = 0.15  # minimum volume so even whisper hits are audible

def hit_volume(peak_g):
    t = (peak_g - VOL_MIN_G) / (VOL_MAX_G - VOL_MIN_G)
    return VOL_FLOOR + (1.0 - VOL_FLOOR) * max(0.0, min(1.0, t))

# ── sync window ───────────────────────────────────────────────────────────────
# When the first H packet arrives, wait up to SYNC_S seconds before playing.
# If the other stick's H packet arrives in that window, cancel the wait and
# fire both sounds at the exact same moment. This compensates for the BLE
# connection-slot offset that always staggers notifications from the two sticks.
SYNC_S       = 0.009  # 9ms — covers 7.5ms BLE slot offset with margin; timeBeginPeriod(1) keeps this precise
_sync_queue  = []     # [(sound_key, peak_g), ...] waiting to fire together
_sync_handle = None   # asyncio TimerHandle
_loop        = None   # event loop reference, set in main()

def _fire_sync():
    global _sync_handle, _sync_queue
    _sync_handle = None
    for key, peak in _sync_queue:
        sound = SFX.get(key)
        if sound:
            ch = sound.play()
            if ch:
                ch.set_volume(hit_volume(peak))
    _sync_queue = []

def _queue_hit(sound_key, peak_g):
    global _sync_handle, _sync_queue
    _sync_queue.append((sound_key, peak_g))
    if _sync_handle is not None:
        # second stick arrived within the window — play everything right now
        _sync_handle.cancel()
        _fire_sync()
    else:
        _sync_handle = _loop.call_later(SYNC_S, _fire_sync)

# ── classification ─────────────────────────────────────────────────────────────
ZONE_LO = -30.0   # snare / R-TOM boundary  (tune if one drum fires too easily)
ZONE_HI =  30.0   # R-TOM / floor tom boundary

hit_counter = 0
recent_hits = deque(maxlen=40)

def classify(roll):
    global hit_counter
    hit_counter += 1
    if roll < ZONE_LO:
        return "SNARE"
    elif roll <= ZONE_HI:
        return "R-TOM"
    else:
        return "FLOOR TOM"

# ── fan visualisation ─────────────────────────────────────────────────────────
#
#   ╱──────────│────────────────────│──────────────╲
#  ╱   SNARE   │      R-TOM         │   FLOOR TOM   ╲
# ╱            │         ◆          │                ╲
# ╱────────────┼────────────────────┼────────────────╲
# -90°        -30°                 +30°             +90°
#
ROLL_LO = -90.0
ROLL_HI =  90.0

def make_fan(arc_w, arc_h):
    def _arc_col(roll_deg):
        return max(0, min(arc_w - 1,
            round((roll_deg - ROLL_LO) / (ROLL_HI - ROLL_LO) * (arc_w - 1))))

    b1 = _arc_col(ZONE_LO)
    b2 = _arc_col(ZONE_HI)
    label_row   = arc_h // 2
    cursor_rows = [arc_h - 2, 1]

    rows = []
    for r in range(arc_h):
        indent  = arc_h - 1 - r
        left_e  = indent
        right_e = arc_w - 1 - indent

        row = [' '] * arc_w

        row[left_e]  = '╱'
        row[right_e] = '╲'

        if r == arc_h - 1:
            for c in range(left_e + 1, right_e):
                row[c] = '─'
            row[b1] = '┼'
            row[b2] = '┼'
        else:
            if left_e < b1 < right_e:
                row[b1] = '│'
            if left_e < b2 < right_e:
                row[b2] = '│'

        if r == label_row:
            def _place(text, lc):
                start = lc - len(text) // 2
                for i, ch in enumerate(text):
                    c = start + i
                    if left_e < c < right_e and row[c] == ' ':
                        row[c] = ch
            _place("SNARE",     (left_e + b1) // 2)
            _place("R-TOM",     (b1 + b2)     // 2)
            _place("FLOOR TOM", (b2 + right_e) // 2)

        for i, s in enumerate(sticks):
            if not s["connected"] or i >= len(cursor_rows):
                continue
            if r == cursor_rows[i]:
                cc = _arc_col(s["smoothed_roll"])
                if left_e < cc < right_e:
                    row[cc] = s["cursor"]

        rows.append('  ' + ''.join(row))

    scale = list('  ' + ' ' * arc_w)
    def _splace(text, col):
        start = 2 + col - len(text) // 2
        for i, ch in enumerate(text):
            p = start + i
            if 0 <= p < len(scale):
                scale[p] = ch
    _splace(f"{ROLL_LO:+.0f}°", 0)
    _splace(f"{ZONE_LO:+.0f}°", b1)
    _splace(f"{ZONE_HI:+.0f}°", b2)
    _splace(f"{ROLL_HI:+.0f}°", arc_w - 1)
    rows.append(''.join(scale))

    return '\n'.join(rows)

# ── render ─────────────────────────────────────────────────────────────────────
def render():
    BOLD  = "\033[1m"
    RESET = "\033[0m"
    YEL   = "\033[93m"
    GRN   = "\033[92m"
    DIM   = "\033[2m"
    CYN   = "\033[96m"

    status_lines = []
    for s in sticks:
        if not s["connected"]:
            status_lines.append(
                f"  {CYN}{s['cursor']}{RESET} {DIM}{s['status']}{RESET}  {s['addr']}"
            )
            continue
        roll      = s["smoothed_roll"]
        zone      = "SNARE" if roll < ZONE_LO else ("FLOOR TOM" if roll > ZONE_HI else "R-TOM")
        sound_key = CYMBAL_MAP[zone] if s["mode"] else zone
        is_last   = zone == s["last_hit"]
        mode_tag  = f" \033[95m[CYMBALS]\033[0m" if s["mode"] else ""
        zone_str  = f"{GRN}{BOLD}{sound_key}{RESET}" if is_last else f"{BOLD}{sound_key}{RESET}"
        status_lines.append(
            f"  {CYN}{s['cursor']}{RESET}  roll:{roll:+.1f}°  accel:{s['amag']:.2f}g  ► {zone_str}{mode_tag}"
        )

    ts     = shutil.get_terminal_size(fallback=(80, 24))
    arc_w  = max(40, ts.columns - 4)
    arc_h  = max(7, ts.lines // 3)
    # rows consumed: arc_h + 1 (scale) + 2 (blank) + len(sticks) + 3 (blank+header+blank)
    n_hits = max(5, ts.lines - arc_h - 6 - len(sticks))

    log = []
    for n, stick_idx, drum, peak in list(recent_hits)[::-1]:
        log.append(
            f"  {YEL}#{n:<3}{RESET} {CYN}{CURSORS[stick_idx]}{RESET} {drum:<12} {peak:.1f}g"
        )
    while len(log) < n_hits:
        log.append(f"  {DIM}─{RESET}")
    log = log[:n_hits]

    total_str = f"total: {hit_counter}" if hit_counter else "none yet"
    print(
        "\033[H\033[J"
        + make_fan(arc_w, arc_h)
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

            if parts[0] == 'S' and len(parts) == 5:
                s["smoothed_roll"] = 0.7 * float(parts[2]) + 0.3 * s["smoothed_roll"]
                s["amag"]          = float(parts[3])
                s["mode"]          = int(parts[4])

            elif parts[0] == 'H' and len(parts) == 10:
                roll      = float(parts[4])   # pre_roll: filtered roll just before the swing
                peak_g    = float(parts[7])
                zone      = classify(roll)    # SNARE / R-TOM / FLOOR TOM
                sound_key = CYMBAL_MAP[zone] if s["mode"] else zone
                s["last_hit"]      = zone
                s["smoothed_roll"] = roll
                recent_hits.append((hit_counter, stick_idx, sound_key, peak_g))
                _queue_hit(sound_key, peak_g)

        except Exception:
            pass

    return callback

# ── connection ────────────────────────────────────────────────────────────────
async def connect_stick(stick_idx):
    s  = sticks[stick_idx]
    cb = make_callback(stick_idx)
    await asyncio.sleep(stick_idx * 12)
    while True:
        s["status"] = "connecting…"
        try:
            async with BleakClient(s["addr"]) as client:
                s["connected"] = True
                s["status"]    = "ok"
                await client.start_notify(CHAR_UUID, cb)
                while client.is_connected:
                    await asyncio.sleep(0.5)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            s["status"] = str(e)[:60]
        finally:
            s["connected"] = False
        await asyncio.sleep(3 + stick_idx * 2)

def _render_thread():
    while True:
        render()
        time.sleep(0.033)

async def main():
    global _loop
    _loop = asyncio.get_running_loop()
    init_audio()
    threading.Thread(target=_render_thread, daemon=True).start()
    await asyncio.gather(
        *(connect_stick(i) for i in range(len(ADDRESSES))),
        return_exceptions=True,
    )

def _run():
    ctypes.windll.winmm.timeBeginPeriod(1)   # 1ms Windows timer resolution
    ctypes.windll.ole32.CoInitializeEx(None, 0x0)
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    try:
        asyncio.run(main())
    finally:
        ctypes.windll.ole32.CoUninitialize()
        ctypes.windll.winmm.timeEndPeriod(1)

t = threading.Thread(target=_run)
t.start()
t.join()
