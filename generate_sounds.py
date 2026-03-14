"""Generate 4 short directional hit sound .wav files with distinct tones."""
import wave
import struct
import math
import os

SAMPLE_RATE = 44100
DURATION = 0.15  # seconds

SOUNDS = {
    "hit_up.wav":    (880, 0.7),   # A5 - high pitch for up
    "hit_down.wav":  (220, 0.7),   # A3 - low pitch for down
    "hit_left.wav":  (523, 0.7),   # C5 - mid-high for left
    "hit_right.wav": (659, 0.7),   # E5 - mid-high for right
}

def generate_tone(filename, freq, volume):
    n_samples = int(SAMPLE_RATE * DURATION)
    with wave.open(filename, 'w') as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SAMPLE_RATE)
        for i in range(n_samples):
            t = i / SAMPLE_RATE
            # Apply fade-out envelope
            envelope = 1.0 - (t / DURATION)
            sample = volume * envelope * math.sin(2 * math.pi * freq * t)
            w.writeframes(struct.pack('<h', int(sample * 32767)))

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    for fname, (freq, vol) in SOUNDS.items():
        path = os.path.join(script_dir, fname)
        generate_tone(path, freq, vol)
        print(f"Created {fname} ({freq} Hz)")
    print("Done!")
