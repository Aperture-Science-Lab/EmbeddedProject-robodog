import sounddevice as sd
import numpy as np

def print_devices():
    print("\n=== Available Audio Devices ===")
    print(sd.query_devices())
    print("===============================\n")

def callback(indata, frames, time, status):
    if status:
        print(status)
    volume_norm = np.linalg.norm(indata) * 10
    # Simple visualizer
    bars = "#" * int(volume_norm * 50)
    print(f"\rVolume: {volume_norm:.4f} | {bars}", end="", flush=True)

print_devices()
print("\n[TEST] Recording... Speak into your mic (Ctrl+C to stop)")

# Default Device ID (None = System Default)
# You can change device=None to device=1, 2, etc. based on the list above
try:
    with sd.InputStream(callback=callback):
        while True:
            sd.sleep(100)
except KeyboardInterrupt:
    print("\n[STOP] Test Complete.")
