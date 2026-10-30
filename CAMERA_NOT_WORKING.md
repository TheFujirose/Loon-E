# Debugging the ZED X camera (no frames, low GPU use)

What we found on 2026-10-29 when `ros2 launch loone launch.py` stopped getting
images from the ZED X, and what to do next. If the camera stops working again,
use the tests below. Each one cuts out one layer (ROS → ZED SDK → Argus →
kernel), so you can tell which layer is broken.

> **Status: unresolved, most likely hardware.** The right-hand image sensor
> (Argus `sensor-id=1`) sends no frames. Everything above it (ROS, configs, ZED
> SDK, GPU, driver) checks out. See [Next steps](#next-steps).

## Hardware / software reference

| Item | Value |
|---|---|
| Camera | ZED-XG 120 mm (ZED X), serial **41775244** |
| Capture card | ZED Link Mono (MAX9296 deserializer), camera on **GMSL port 0** |
| Driver | `stereolabs-zedlink-mono` 1.4.2-SL-MAX9296-L4T36.5.0 |
| Jetson | Orin Nano Super Developer Kit, L4T R36.5.0 |
| ZED SDK | 5.4.0 (`~/.loon_env_setup.bash` says 5.2.3, which is out of date) |
| CUDA | 12.6 |
| Video devices | `/dev/video0` (`zedx 10-0028`), `/dev/video1` (`zedx 10-0020`) |

How the camera stack fits together, top to bottom:

```
ros2 launch loone launch.py
  └─ zed_wrapper (component_container_isolated)   ← ROS
       └─ ZED SDK (libsl_zed.so)                  ← stereo sync, depth, object detection
            └─ nvargus-daemon (Argus)             ← NVIDIA camera service, owns /dev/video*
                 └─ kernel: sl_zedx, sl_max9295/9296, tegra VI/NVCSI
                      └─ GMSL cable → camera (2 image sensors)
```

## Symptoms

- jtop shows low GPU use with the launch running.
- The ZED node log shows any of:
  - `CAMERA STREAM FAILED TO START`
  - `checkStreamStatus: no NEW_FRAME state within 20000ms budget`
  - `Corrupted frames were detected. Please verify that your camera is properly connected.`
  - `[ZED-Argus][Critical] Reboot has failed for CAM0`
  - Floods of `(Argus) Error InvalidState: Receive thread is not running cannot send`
    or `(Argus) Error Timeout`
- `journalctl -u nvargus-daemon` shows `Error AlreadyAllocated: Device 0 (of 1) is in use`.

## What we found

### Root cause: sensor 1 sends no frames

The ZED X is a stereo camera. The ZED SDK needs **both** image sensors streaming
in sync. Tested directly through Argus, with no ZED SDK or ROS involved:

| Test | Result |
|---|---|
| Kernel detects both sensors at boot | OK (`ZED-X sensor initialisation done` for `10-0020` and `10-0028`) |
| GMSL link | OK (`camera pipeline operational`, `Serializer pipeline operational`) |
| Argus, sensor 0 alone | **Streams**: 60 frames in about 3 s |
| Argus, sensor 1 alone | **No frames**: times out after 16 s, `NvBufSurfaceFromFd Failed` |
| Argus, both sensors | Fails (because of sensor 1) |
| Kernel log during the sensor 1 test | **No CSI/CRC/VI errors at all** |
| ZED SDK alone (`pyzed`, SVGA @ 30) | Fails: `no NEW_FRAME ... 20000ms`, `Corrupted frames` |
| `ZED_Diagnostic -c` with Argus freshly restarted | Camera `"state": "Not Available"`, no sensors listed |

No kernel errors means data from sensor 1 isn't arriving damaged: it isn't
arriving at all. That points to the camera (sensor 1 or its serializer channel)
or the GMSL connection, not to the Jetson.

### Ruled out

- **Our code and configs.** The ZED SDK fails the same way with no ROS running.
- **Resolution / frame rate.** Changing to SVGA @ 30 fps didn't help.
- **`.bashrc` / environment.** CUDA paths are correct. The wrapper is built
  against the installed SDK 5.4.0.
- **GPU / CUDA.** `ZED_Diagnostic` shows CUDA working at 1020 MHz, and the NEURAL
  and NEURAL LIGHT depth models are already optimized.
- **Another program holding the camera.** `sudo fuser -v /dev/video0 /dev/video1`
  showed only `nvargus-daemon`. No second launch, systemd service, cron job or
  autostart starts the camera.

## Things that make it look worse than it is

These caused most of the confusion while debugging. Know them before you start.

### 1. A failed open locks Argus until it's restarted

When the ZED SDK gets no frames, it restarts `nvargus-daemon` on its own (look
for `Stopped/Started Argus daemon` in the journal with no sudo command from you),
then gives up. The dead capture session stays held, and **every later attempt**
(ROS, `pyzed`, `gst-launch`, `ZED_Diagnostic`) fails instantly with
`Device 0 is in use` / `Failed to create CaptureSession`, even on a sensor that
works.

**Always restart Argus before each test:**

```bash
sudo systemctl restart nvargus-daemon && sleep 3
```

### 2. Don't change the power mode while the camera is running

Switching `nvpmodel` (or running `jetson_clocks`) at runtime changes clocks under
the capture pipeline and can break the stream until a reboot. Set the power mode
first, reboot, then start the camera. This board supports modes 0 (15W),
1 (25W, default) and 2 (MAXN_SUPER). **There is no mode 3.** It's currently in
**MAXN_SUPER**.

### 3. `tools/reset.sh` / `tools/fast_reset` open the camera themselves

They finish with `ZED_Explorer -a` to check the camera. They also unload and
reload the ZED driver. Running one right before a launch adds another
open/restart cycle. On a clean boot, don't run them before the first launch.

### 4. `IMU_Daemon` doesn't come back by itself

`IMU_Daemon` depends on `zed_x_daemon`. If `zed_x_daemon` is restarted (by
`ZED_Diagnostic` or by the SDK's automatic recovery), systemd stops `IMU_Daemon`
and leaves it stopped. Video still works, but IMU data (which `navsat_transform`
needs) is gone. Check with `systemctl is-active IMU_Daemon` and start it with
`sudo systemctl start IMU_Daemon`. `tools/check.sh` checks this too.

### 5. `ZED_Diagnostic --dmesg` doesn't test the camera

`--dmesg` only saves the kernel log (as `dmesg*.log`, owned by root, in the
current directory). The full test is `sudo ZED_Diagnostic -c`, run on a freshly
restarted Argus. It writes `ZED_Diagnostic_Results.json`. Its "OK" lines only mean
each check *ran*. Read `USB Camera → GMSLList[0].state` in the JSON, where
`"Not Available"` means the camera failed.

## Quick check: `tools/CAMERA_DEBUG.py`

Runs every check below automatically (restarting Argus before each capture
test) and prints a chart of the results with next steps:

```bash
tools/CAMERA_DEBUG.py               # everything, 2-4 min (asks for sudo)
tools/CAMERA_DEBUG.py --skip-sdk    # skip the slow ZED SDK test
tools/CAMERA_DEBUG.py --no-capture  # read-only checks, restarts nothing
```

It saves a full log to `/tmp/camera_debug_<time>.log`. Attach that to a
Stereolabs ticket. Argus `sensor-id=0` is the **left** sensor
(`zedx_left_0@28`, `/dev/video1`). `sensor-id=1` is the **right** sensor
(`zedx_right_0@20`, `/dev/video0`).

## Tests

To run the tests by hand, run them in this order. Stop ROS first (`Ctrl+C` the launch and check with
`pgrep -af component_container` that nothing is left).

### Test 1: each sensor through Argus (no ZED SDK)

```bash
for id in 0 1; do
  echo "== sensor $id"
  sudo systemctl restart nvargus-daemon && sleep 3
  timeout 25 gst-launch-1.0 -e nvarguscamerasrc sensor-id=$id num-buffers=60 \
    ! 'video/x-raw(memory:NVMM)' ! fakesink sync=false 2>&1 \
    | grep -E "Error|Done Success|Execution ended"
done
sudo systemctl restart nvargus-daemon
```

| What you see | Meaning |
|---|---|
| `Execution ended after 0:00:02-3...` and `Done Success` | Sensor is streaming |
| `Execution ended after 0:00:16...` and `NvBufSurfaceFromFd Failed` | **No frames from this sensor** |
| `Execution ended after 0:00:00.00...` and `Failed to create CaptureSession` | Argus is locked from an earlier failure. Restart it and rerun. |

### Test 2: kernel view

Run right after a failed sensor test:

```bash
sudo dmesg | grep -iE "vi5|nvcsi|camrtc|corr|chansel|zedx|max929" | tail -20
```

- CSI/CRC/`corr`/`chansel` errors: data arrives damaged, so suspect the cable or connector.
- Nothing but the boot-time probe lines: no data arrives, so suspect the sensor
  (this is what we saw).

### Test 3: ZED SDK alone (no ROS)

```bash
sudo systemctl restart nvargus-daemon && sleep 3
sudo ZED_Diagnostic -c
```

Then check `ZED_Diagnostic_Results.json` → `USB Camera` → `GMSLList[0].state`.

### Test 4: the full launch

Only once tests 1 and 3 pass:

```bash
ros2 launch loone launch.py
```

## Next steps

1. **Power off fully** (unplug, not just reboot). Reseat the GMSL cable at the
   camera and at the ZED Link card. Look for bent pins, kinks, moisture or
   corrosion. It's a boat.
2. Boot, **don't** run `tools/reset.sh`, and run **Test 1** (or `tools/CAMERA_DEBUG.py`). If sensor 1 streams, it's
   fixed. Go to step 6.
3. Move the camera to **GMSL port 1** on the ZED Link card and run Test 1 again.
   If it works there, port 0 or the card is the problem.
4. If you have a spare GMSL cable, or another ZED X, swap it in and run Test 1:
   - Another camera works on this setup: **our camera is faulty.**
   - Another camera fails the same way: suspect the ZED Link card or the Jetson.
5. If sensor 1 still sends nothing, **contact Stereolabs support**
   (through stereolabs.com → Support). Send them:
   - Model and serial: ZED-XG 120 mm, SN 41775244
   - `ZED_Diagnostic_Results.json` from Test 3
   - Platform: Orin Nano Super, L4T R36.5.0, ZED SDK 5.4.0, ZED Link Mono driver 1.4.2
   - The key finding: *Argus sensor-id=0 streams; sensor-id=1 times out with no
     kernel CSI errors; both sensors probe successfully at boot.*
6. **Once the camera works, finish the GPU question.** See below.

## After the camera works: getting the GPU busy

The original question was why the GPU looked idle. Two things to know:

- **Object detection only runs when something subscribes to it.** The wrapper
  skips inference when `/zedx/zed_node/obj_det/objects` has no subscribers
  (`processDetectedObjects()` in
  `src/zedx/zed_components/src/zed_camera/src/zed_camera_component_objdet.cpp`).
  The `task` node is the only subscriber, and it's commented out of `launch.py`.
  To check:
  ```bash
  ros2 topic echo /zedx/zed_node/obj_det/objects --no-arr > /dev/null
  ```
  GPU load in jtop should jump as soon as this starts. Images, depth and point
  clouds work the same way: they're only produced when something subscribes.
- **Power mode.** 15W caps the GPU at 612 MHz. MAXN_SUPER allows 1020 MHz.
  It's now set to MAXN_SUPER, and the setting survives reboots.

Check the object detection rate with `ros2 topic echo /diagnostics`, under
"Object detection: Mean Frequency".

## Changes made during this debug

| File | Change | Keep? |
|---|---|---|
| `src/zedx/zed_wrapper/config/zedx.yaml` | `grab_resolution` HD1080 → **SVGA**, `grab_frame_rate` 60 → **30** | Test setting. Revert to HD1080 / 60 once the camera works, if you want full resolution. It's in the `src/zedx` submodule and not committed. |
| `src/loone/loone/hardware_node.py` | Only call `rclpy.shutdown()` if `rclpy.ok()`, which fixes the `rcl_shutdown already called` traceback on Ctrl+C | Yes. Needs `colcon build --packages-select loone`. |

Clean-up: `dmesg*.log` and `ZED_Diagnostic_Results.json` in the repo root were
created by `ZED_Diagnostic`. They're owned by root and not tracked by git. Keep
the JSON for the Stereolabs ticket, then `sudo rm dmesg*.log`.
