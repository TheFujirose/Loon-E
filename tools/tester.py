"""Standalone launcher ESC probe. Stop hardware_node first (same I2C bus).

Usage: python3 launcher_probe.py [channel]   (default 3)
Arms the ESC with 1500 us for 3 s, then steps through pulses; Ctrl+C returns to neutral.
"""
import sys
import time

import board
import busio
from adafruit_pca9685 import PCA9685

ch = int(sys.argv[1]) if len(sys.argv) > 1 else 3
pca = PCA9685(busio.I2C(board.SCL, board.SDA), address=64)
pca.frequency = 50
out = pca.channels[ch]


def pulse(us):
    out.duty_cycle = int(us * pca.frequency / 1_000_000 * 0xFFFF)
    print(f"ch{ch}: {us} us (duty 0x{out.duty_cycle:04X})", flush=True)


try:
    pulse(1500)
    print("Arming: plug in / power the ESC now if it isn't, listen for the beeps...")
    time.sleep(3)
    for us in (1600, 1700, 1800, 1500, 1400, 1300, 1500):
        pulse(us)
        time.sleep(2)
finally:
    pulse(1500)
    time.sleep(0.5)
    pca.deinit()
