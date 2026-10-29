"""Work out the PCA9685's real oscillator speed for config.yaml's pca_clock.

Stop hardware_node first (same I2C bus). Puts a 50% duty square wave on a spare
channel (no ESC/servo attached!) at the configured frequency. Measure its
frequency with a multimeter in Hz mode, a scope or a logic analyser, enter it,
and the script prints the pca_clock value to use.

Usage: python3 pca_clock_cal.py [channel]   (default 7)
"""
import sys

import board
import busio
from adafruit_pca9685 import PCA9685

NOMINAL = 25_000_000
FREQ = 50

ch = int(sys.argv[1]) if len(sys.argv) > 1 else 7
pca = PCA9685(busio.I2C(board.SCL, board.SDA), address=64, reference_clock_speed=NOMINAL)
try:
    pca.frequency = FREQ
    expected = pca.frequency  # what the prescaler should give with a 25 MHz clock
    pca.channels[ch].duty_cycle = 0x8000
    print(f"ch{ch}: square wave, expected {expected:.3f} Hz if the clock were exactly 25 MHz.")
    measured = float(input("Measured frequency (Hz): "))
    clock = round(NOMINAL * measured / expected)
    error = (measured / expected - 1) * 100
    print(f"Clock error {error:+.2f}%  -> neutral 1500 us would really be {1500 / (1 + error / 100):.0f} us")
    print(f"Set in config.yaml:  pca_clock: {clock}")
finally:
    pca.channels[ch].duty_cycle = 0
    pca.deinit()
