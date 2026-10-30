# Debugging the PWM hardware (ESCs, servos)

How to check the PCA9685 PWM board and the Blue Robotics Basic ESCs when a motor
won't move or behaves wrongly. Both scripts below talk straight to the PCA9685
over I2C, without ROS.

> **Before running either script:** stop `hardware_node` (it uses the same I2C
> bus), keep hands and props clear, and make sure the boat is secured.
>
> The `Carrier board is not from a Jetson Developer Kit` / `Jetson.GPIO ... not
> been verified` warnings and the `Exiting... Cleaning up pins` lines on exit are
> normal on this board and can be ignored.

## Hardware reference

PCA9685 at I2C address `0x40`, running at 50 Hz:

| Channel | Device | Pulse range (µs) |
|---|---|---|
| 0 | Left propeller ESC | 1120–1880 |
| 1 | Right propeller ESC | 1120–1880 |
| 2 | Ball carriage servo (positional, CCW with longer pulse) | 500–2500 |
| 3 | Ball launcher ESC | 1100–1900 |
| 4 / 5 | Status LED red / green | on/off |
| 7 | Free (used for clock calibration) | — |

The pulse ranges, PWM frequency and `pca_clock` are set in
`src/loone/config/config.yaml` (for `hardware_node`) and passed directly to
`HardwareController(...)` in `src/loone/loone/test.py`.

**Blue Robotics Basic ESC signal**
([product page](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/)):

- 1100 µs full reverse, 1500 µs stop, 1900 µs full forward
- Dead zone: 1500 ± 25 µs. Anything inside it counts as stop.
- **Arming:** the ESC must see 1500 µs for a few seconds after it powers on. It
  plays rising tones at power-on and a final beep once armed. Until it arms, it
  ignores throttle.
- Bidirectional by default.

## Tool 1: `tools/tester.py`, raw ESC probe

```bash
python3 tools/tester.py [channel]    # default channel 3 (launcher)
```

It sends 1500 µs for 3 s (arming), then steps through
1600 → 1700 → 1800 → 1500 → 1400 → 1300 → 1500 µs, 2 s each, printing each
pulse. Ctrl+C returns the channel to 1500 µs before exiting.

Use it to answer one question: **is the problem hardware or code?** It bypasses
`HardwareController` entirely.

| What you see / hear | Meaning |
|---|---|
| Beeps, then the motor spins forward and reverse | Wiring and ESC are fine. The problem is in the code. |
| Beeps but no spin | ESC is armed. Check the three motor wires from the ESC to the motor. |
| No beeps | ESC isn't powered or isn't getting signal. Check the battery to the ESC, which channel the signal plug is on, and the ground between PCA9685 and ESC. Try another channel (e.g. `tester.py 0`) to prove the board itself works. |
| Motor creeps or twitches at "1500 µs" | Board clock is off. See Tool 2. |

> `tester.py` uses the PCA9685's *nominal* 25 MHz clock, not the calibrated
> `pca_clock`, so its pulses are off by the board's clock error (about 3% on the
> current board, so its "1500 µs" is really about 1456 µs). That's fine for a
> spins / doesn't-spin check, but don't use it to judge exact stop or full
> throttle.

## Tool 2: `tools/pca_clock_cal.py`, board clock calibration

The PCA9685's built-in clock is nominally 25 MHz, but it varies by a few percent
from board to board, and **every pulse width scales with that error**. Each 1% of
error moves the 1500 µs stop signal by about 15 µs, so anything more than about
1.6% puts "stop" outside the ESC's dead zone. Symptoms: ESCs that won't arm,
motors creeping when they should be stopped, or less than full throttle.

```bash
python3 tools/pca_clock_cal.py [channel]    # default channel 7
```

1. Make sure **nothing is plugged into the channel** (default ch7 is free).
2. The script outputs a 50% square wave and prints the frequency it *expects*
   (about 50.03 Hz).
3. Measure the real frequency on that channel's signal pin with a multimeter in
   Hz mode, an oscilloscope or a logic analyser, and type it in.
4. It prints the clock error, where "1500 µs" really lands, and the value to use:

   ```
   Measured frequency (Hz): 51.55
   Clock error +3.04%  -> neutral 1500 us would really be 1456 us
   Set in config.yaml:  pca_clock: 25760154
   ```

5. Put that value in **both** places:
   - `src/loone/config/config.yaml` → `pca_clock: <value>`
   - `src/loone/loone/test.py` → `HardwareController(..., pca_clock=<value>)`
6. Rebuild (below).

The value only depends on the PCA9685 board, so redo this only if the board is
replaced. **Current board: 25760154 Hz** (measured 51.55 Hz at 50 Hz).

## Rebuild after any change

ROS runs the installed copy under `install/`, not `src/`, so code and config edits
do nothing until you rebuild:

```bash
colcon build --packages-select loone   # earlier builds were done as root, so you may need sudo
source install/setup.bash
```

Check that the `PCA9685 initialized at 50 Hz (clock ... Hz)` log line shows the
calibrated clock.

## Step-by-step: a motor doesn't move

1. **Stop `hardware_node`.**
2. **Run `tester.py <channel>`** and follow the table in Tool 1.
   - Nothing on the expected channel but it works on another: either the wire is
     on the wrong channel or the code uses the wrong one (see the channel map in
     `HardwareController`).
3. **If `tester.py` works but the robot code doesn't:**
   - Rebuild. The `install/` copy may be out of date.
   - Check that the expected `pca_clock` is set (Tool 2).
   - Look at the terminal (stderr), not only the ROS log. Ball launcher sequences
     run on a background thread in `hardware_node`, and an exception there (e.g.
     a bad argument to `load()` / `launch()`) is printed to stderr and the motor
     simply never moves.
   - The ESC needs neutral for a few seconds after power-on. `HardwareController`
     sets neutral at start-up, so power the ESCs **before** starting the code, or
     restart the code after powering them.
   - A small `power` (e.g. `launch(power=0.1)`, only about 40 µs from stop) may be
     too little to spin the wheel under load. Try a higher value.
4. **If the motor creeps, won't arm reliably, or seems weak:** run
   `pca_clock_cal.py` (Tool 2) and update `pca_clock`.
