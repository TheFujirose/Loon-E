#!/usr/bin/env python3
"""ZED X camera debug tool.

Checks every layer of the camera stack, from the device tree up to the ZED SDK,
then prints a chart of the results and what they mean. See CAMERA_NOT_WORKING.md
for the background.

Needs root: a failed capture leaves Argus holding the camera until
nvargus-daemon is restarted, so it is restarted before every capture test.
Started as a normal user, the script re-runs itself with sudo.

    tools/CAMERA_DEBUG.py               # everything (2-4 min)
    tools/CAMERA_DEBUG.py --skip-sdk    # skip the slow ZED SDK test
    tools/CAMERA_DEBUG.py --no-capture  # read-only checks, restarts nothing

Stop the ROS launch first, or the capture tests will fight it for the camera.
"""

import argparse
import datetime
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import textwrap
import threading
import time
from dataclasses import dataclass, field

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # project root
ZEDX_YAML = os.path.join(ROOT, 'src', 'zedx', 'zed_wrapper', 'config', 'zedx.yaml')
DT_CAM = '/proc/device-tree/tegra-camera-platform/modules'
DT_MUX = '/sys/firmware/devicetree/base/bus@0/cam_i2cmux'
ZED_HEADER = '/usr/local/zed/include/sl/Camera.hpp'

SERVICES = ['nvargus-daemon', 'zed_x_daemon', 'IMU_Daemon']
MODULES = ['sl_zedx', 'sl_max9296', 'sl_max9295']
RESOLUTIONS = {'HD1200': (1920, 1200), 'HD1080': (1920, 1080), 'SVGA': (960, 600)}

# Programs that open the camera through Argus. Argus itself holds /dev/video*
# on their behalf, so they never show up in fuser.
CAMERA_USERS = re.compile(
    r'component_container|zed_wrapper|zed_camera|ZED_Explorer|ZED_Depth_Viewer|'
    r'ZED_Diagnostic|ZED_Sensor_Viewer|ZED_Studio|ZEDfu|nvarguscamerasrc|ros2 launch')

# nvargus-daemon probes every camera slot in the device tree; the empty ones
# always log these (same list as check.sh).
ARGUS_BENIGN = re.compile('|'.join([
    'Model Name is NULL', 'ModuleNotPresent', 'Failed to init camera sub module',
    'PCL Open Failed', 'Sensor could not be opened', 'SCF: Error BadParameter',
    'serial no file already exists', r'Failed ImagerGUID [2-9]', 'NvPcl',
    'V4L2Device not available', 'Could not map module', 'No module data',
    'OFParser', 'PowerServiceCore']))
KERNEL_CAPTURE_ERR = re.compile(
    r'vi5|nvcsi|camrtc|chansel|corr|csi.*(err|fail)|capture.*(err|fail|timeout)|'
    r'(zedx|max929\d).*(err|fail|timeout)', re.I)
# 9-0048 is the unused second deserializer slot on the ZED Link Mono card.
KERNEL_BENIGN = re.compile(
    r'probe of 9-0048 failed|module verification failed|cyclic dependency|'
    r'\bbound\b|unbind|success', re.I)

PYZED_TEST = '''
import json, sys, time
import pyzed.sl as sl
res, fps, frames = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
init = sl.InitParameters()
init.camera_resolution = getattr(sl.RESOLUTION, res, sl.RESOLUTION.AUTO)
init.camera_fps = fps
init.depth_mode = sl.DEPTH_MODE.NONE  # only test capture, not the depth model
cam = sl.Camera()
t0 = time.time()
err = cam.open(init)
out = {"open": str(err), "open_s": round(time.time() - t0, 1)}
if err == sl.ERROR_CODE.SUCCESS:
    info = cam.get_camera_information()
    cfg = info.camera_configuration
    out.update(model=str(info.camera_model), serial=info.serial_number,
               width=cfg.resolution.width, height=cfg.resolution.height, fps=cfg.fps)
    rt = sl.RuntimeParameters()
    ok = 0
    t0 = time.time()
    for _ in range(frames):
        if cam.grab(rt) == sl.ERROR_CODE.SUCCESS:
            ok += 1
    out.update(grabbed=ok, frames=frames, grab_s=round(time.time() - t0, 1))
    cam.close()
print("RESULT " + json.dumps(out), flush=True)
'''

TTY = sys.stdout.isatty()
STATUS_COLOR = {'PASS': '32', 'FAIL': '31', 'WARN': '33', 'INFO': '36', 'SKIP': '90'}
LOG = []


def color(text, code):
    return f'\033[{code}m{text}\033[0m' if TTY else text


def badge(status):
    return color(f'{status:<4}', STATUS_COLOR[status])


def log(text):
    LOG.append(text)


@dataclass
class Result:
    name: str
    status: str
    details: str
    extra: dict = field(default_factory=dict)


RESULTS = {}  # key -> Result, in run order


def record(key, name, status, details, **extra):
    """Store a result and print it as a one-line progress update."""
    RESULTS[key] = Result(name, status, details, extra)
    log(f'[{status}] {name}: {details}')
    line = f'{name}: {details}'
    room = shutil.get_terminal_size((100, 20)).columns - 9
    if len(line) > room:
        line = line[:room - 1] + '…'
    print(f'  {badge(status)}  {line}')


class Spinner:
    """Shows 'label', 'label.', 'label..', 'label...' while a step runs."""

    def __init__(self, label):
        self.label = label
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def _spin(self):
        n = 0
        while True:
            secs = time.monotonic() - self.t0
            sys.stdout.write(f'\r\033[K  {self.label}{"." * n:<3}  ({secs:.0f}s)')
            sys.stdout.flush()
            n = (n + 1) % 4
            if self._stop.wait(0.4):
                break

    def __enter__(self):
        self.t0 = time.monotonic()
        if TTY:
            self._thread.start()
        else:
            print(f'  {self.label}...', flush=True)
        return self

    def __exit__(self, *exc):
        if TTY:
            self._stop.set()
            self._thread.join()
            sys.stdout.write('\r\033[K')
            sys.stdout.flush()


def run(cmd, timeout=30, user=None, log_output=True):
    """Run a command; return (exit code or None on timeout, combined output, seconds)."""
    if user:
        cmd = ['sudo', '-u', user, '-H'] + cmd
    t0 = time.monotonic()
    try:
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, errors='replace', timeout=timeout)
        code, out = proc.returncode, proc.stdout
    except subprocess.TimeoutExpired as e:
        code = None
        out = e.stdout.decode(errors='replace') if isinstance(e.stdout, bytes) \
            else (e.stdout or '')
    except FileNotFoundError:
        code, out = 127, f'{cmd[0]}: command not found'
    secs = time.monotonic() - t0
    if log_output:
        log(f'$ {" ".join(cmd)}  (exit {code}, {secs:.1f}s)\n{out.rstrip()}\n')
    return code, out, secs


def read_dt(path):
    try:
        with open(path, 'rb') as f:
            return f.read().replace(b'\0', b'').decode(errors='replace')
    except OSError:
        return ''


def kernel_log():
    code, out, _ = run(['dmesg'], log_output=False)
    if code != 0 or not out.strip():
        code, out, _ = run(['journalctl', '-k', '-b', '--no-pager', '-o', 'cat'],
                           log_output=False)
    return out.splitlines()


def restart_argus():
    """Restart Argus so a dead session from an earlier failure can't block the test."""
    run(['systemctl', 'restart', 'nvargus-daemon'])
    time.sleep(3)


# Argus sensor-id N follows device tree moduleN; filled in by check_device_tree().
SENSOR_NAMES = {}
# I2C address -> 'left'/'right', from the device tree.
ADDR_SIDE = {}


def sensor_label(i):
    return f'sensor {i} ({SENSOR_NAMES[i]})' if i in SENSOR_NAMES else f'sensor {i}'


# ----------------------------------------------------------------- static checks

def check_versions():
    rel = read_dt('/etc/nv_tegra_release').splitlines()
    m = re.search(r'R(\d+).*REVISION: ([\d.]+)', rel[0]) if rel else None
    l4t = f'R{m[1]}.{m[2]}' if m else '?'
    try:
        with open(ZED_HEADER) as f:
            v = dict(re.findall(r'#define ZED_SDK_(MAJOR|MINOR|PATCH)_VERSION (\d+)', f.read()))
        sdk = f'{v["MAJOR"]}.{v["MINOR"]}.{v["PATCH"]}'
    except (OSError, KeyError):
        sdk = 'not found'
    _, out, _ = run(['dpkg-query', '-W', '-f=${db:Status-Abbrev} ${Package} ${Version}\n',
                     'stereolabs-zed*'])
    pkgs = [ln[4:] for ln in out.splitlines() if ln.startswith('ii ')]
    driver = ', '.join(pkgs) or 'no stereolabs-zed* driver package'
    status = 'INFO' if pkgs and sdk != 'not found' else 'FAIL'
    return status, f'L4T {l4t}, ZED SDK {sdk}, {driver}'


def check_device_tree():
    if not os.path.isdir(DT_CAM):
        return 'FAIL', 'no tegra-camera-platform node: the ZED Link device tree overlay ' \
                       'is not loaded'
    slots = []
    for m in sorted(glob.glob(os.path.join(DT_CAM, 'module*'))):
        idx = int(re.search(r'\d+$', m)[0])
        node = os.path.basename(read_dt(os.path.join(m, 'drivernode0', 'sysfs-device-tree')))
        slots.append((idx, read_dt(os.path.join(m, 'badge')), read_dt(os.path.join(m, 'status')),
                      node))
        if node:
            SENSOR_NAMES[idx] = node
    for path in glob.glob(os.path.join(DT_MUX, 'i2c@*', 'zedx_*@*')):
        m = re.match(r'zedx_(left|right)_\d+@([0-9a-f]+)$', os.path.basename(path))
        if m:
            ADDR_SIDE[int(m[2], 16)] = m[1]
    zed = [s for s in slots if s[1].startswith('zedx') and s[2] == 'okay']
    first = {s[0]: s for s in zed if s[0] in (0, 1)}
    if len(first) < 2:
        return 'FAIL', f'camera slots 0 and 1 missing or disabled ({len(zed)} ZED slots found)'
    return 'PASS', (f'{len(zed)} ZED camera slots enabled; module0 = {first[0][3]}, '
                    f'module1 = {first[1][3]}')


def check_modules():
    loaded = {ln.split()[0] for ln in read_dt('/proc/modules').splitlines() if ln}
    missing = [m for m in MODULES if m not in loaded]
    if missing:
        return 'FAIL', f'not loaded: {", ".join(missing)} (try: sudo service driver_zed_loader ' \
                       'start)'
    return 'PASS', ', '.join(MODULES) + ' loaded'


def check_sensor_probe(klog):
    probed = {}
    for line in klog:
        m = re.search(r'\bzedx (\d+-[0-9a-f]{4}): zedx_probe: Serial Number : (\d+)', line)
        if m:
            probed[m[1]] = m[2]
    ok = {dev for line in klog
          for dev in re.findall(r'\bzedx (\d+-[0-9a-f]{4}): zedx_probe: success', line)}
    log('Camera kernel lines:\n' + '\n'.join(
        ln for ln in klog if re.search(r'zedx|max929|GMSL|gmsl', ln)) + '\n')
    names = [f'{ADDR_SIDE.get(int(d.split("-")[1], 16), "?")} ({d})' for d in sorted(ok)]
    serials = ', '.join(sorted(set(probed.values()))) or '?'
    if len(ok) < 2:
        return 'FAIL', f'only {len(ok)} of 2 sensors probed ({", ".join(names) or "none"}); ' \
                       'check the GMSL cable and camera power'
    return 'PASS', f'{" and ".join(names)} probed, SN {serials}'


def check_gmsl(klog):
    text = '\n'.join(klog)
    ports = sorted(set(re.findall(r'Camera connected to GMSL port (\d)', text)))
    deser = 'camera pipeline operational' in text
    ser = 'Serializer pipeline operational' in text
    model = re.search(r'(\w+) camera identified via eeprom', text)
    missing = [n for n, ok in (('camera on a GMSL port', ports),
                               ('deserializer pipeline', deser),
                               ('serializer pipeline', ser)) if not ok]
    if missing:
        return 'FAIL', 'kernel log is missing: ' + ', '.join(missing)
    return 'PASS', (f'{model[1] if model else "camera"} on GMSL port {"/".join(ports)}; '
                    'deserializer and serializer pipelines operational')


def check_video_devices():
    devs = []
    for path in sorted(glob.glob('/sys/class/video4linux/video*')):
        name = read_dt(os.path.join(path, 'name')).strip()
        m = re.search(r'zedx (\d+-([0-9a-f]{4}))', name)
        if m:
            side = ADDR_SIDE.get(int(m[2], 16), '?')
            devs.append(f'/dev/{os.path.basename(path)} = {side} ({m[1]})')
    if len(devs) < 2:
        return 'FAIL', f'expected 2 zedx video devices, found {len(devs)}: ' \
                       f'{", ".join(devs) or "none"}'
    return 'PASS', ', '.join(devs)


def check_services():
    states = {}
    for svc in SERVICES:
        _, out, _ = run(['systemctl', 'is-active', svc])
        states[svc] = out.strip() or 'unknown'
    _, loader, _ = run(['systemctl', 'is-failed', 'driver_zed_loader'])
    details = ', '.join(f'{s} {st}' for s, st in states.items())
    if loader.strip() == 'failed':
        details += ', driver_zed_loader FAILED'
    down = [s for s, st in states.items() if st != 'active']
    if down or loader.strip() == 'failed':
        return 'FAIL', details, {'down': down}
    return 'PASS', details, {'down': []}


def check_power():
    _, out, _ = run(['nvpmodel', '-q'])
    m = re.search(r'NV Power Mode: (\S+)', out)
    mode = m[1] if m else '?'
    freqs = glob.glob('/sys/class/devfreq/*.gpu/max_freq')
    gpu = f', GPU max {int(read_dt(freqs[0])) // 1000000} MHz' if freqs else ''
    _, journal, _ = run(['journalctl', '-b', '--no-pager', '_COMM=sudo'], log_output=False)
    changes = [f'{ln.split()[2]} ({c.strip()})' for ln in journal.splitlines()
               for c in re.findall(r'COMMAND=\S*(nvpmodel -m \d+|jetson_clocks)', ln)]
    details = f'{mode}{gpu}'
    if changes:
        return 'WARN', (f'{details}; changed this boot: {", ".join(changes[-3:])}. '
                        'Changing it while the camera runs breaks capture; reboot to be sure')
    return 'INFO', details


def check_holders():
    _, out, _ = run(['fuser', '-v'] + glob.glob('/dev/video*') + glob.glob('/dev/media*'))
    holders = {f'{m[2]} (pid {m[1]})'
               for m in re.finditer(r'(\d+)\s+[Ffrwcem.]{5}\s+(\S+)\s*$', out, re.M)
               if m[2] != 'nvargus-daemon'}
    _, ps, _ = run(['ps', '-eo', 'pid=,args='], log_output=False)
    me = {os.getpid(), os.getppid()}
    for line in ps.splitlines():
        pid, _, args = line.strip().partition(' ')
        if CAMERA_USERS.search(args) and int(pid) not in me and 'CAMERA_DEBUG' not in args:
            holders.add(f'{args.split()[0].split("/")[-1]} (pid {pid})')
    if holders:
        return 'FAIL', 'camera in use by: ' + ', '.join(sorted(holders)), \
            {'holders': sorted(holders)}
    return 'PASS', 'only nvargus-daemon has /dev/video* open; no ROS or ZED tools running', \
        {'holders': []}


def sensor_modes():
    dev = next((f'/dev/{os.path.basename(p)}'
                for p in sorted(glob.glob('/sys/class/video4linux/video*'))
                if 'zedx' in read_dt(os.path.join(p, 'name'))), None)
    if not dev:
        return {}
    _, out, _ = run(['v4l2-ctl', '-d', dev, '--list-formats-ext'])
    modes, size = {}, None
    for line in out.splitlines():
        m = re.search(r'Size: Discrete (\d+)x(\d+)', line)
        if m:
            size = (int(m[1]), int(m[2]))
            modes.setdefault(size, [])
            continue
        m = re.search(r'\(([\d.]+) fps\)', line)
        if m and size:
            modes[size].append(round(float(m[1])))
    return modes


def fmt_modes(modes):
    return ', '.join(f'{w}x{h} @ {"/".join(map(str, sorted(f)))} fps'
                     for (w, h), f in modes.items())


def read_config():
    try:
        with open(ZEDX_YAML) as f:
            text = f.read()
    except OSError:
        return None, None
    res = re.search(r"^\s*grab_resolution:\s*'?(\w+)'?", text, re.M)
    fps = re.search(r'^\s*grab_frame_rate:\s*(\d+)', text, re.M)
    return (res[1] if res else None), (int(fps[1]) if fps else None)


def check_config(modes, res, fps):
    where = os.path.relpath(ZEDX_YAML, ROOT)
    if res is None or fps is None:
        return 'WARN', f'could not read grab_resolution / grab_frame_rate from {where}'
    if res not in RESOLUTIONS:
        return 'INFO', f'{res} @ {fps} fps in {where} (SDK picks the sensor mode)'
    size = RESOLUTIONS[res]
    if modes and size not in modes:
        return 'FAIL', f'{res} ({size[0]}x{size[1]}) is not a sensor mode ({fmt_modes(modes)})'
    native = modes.get(size, [])
    if native and fps not in native:
        return 'WARN', (f'{res} @ {fps} fps in {where}: {fps} fps is not a native rate for '
                        f'{size[0]}x{size[1]} (sensor: {"/".join(map(str, native))} fps)')
    return 'PASS', f'{res} @ {fps} fps in {where}'


# ----------------------------------------------------------------- capture tests

def argus_capture(ids, frames):
    """Capture through Argus only (no ZED SDK). Returns per-sensor results and log errors."""
    restart_argus()
    before = len(kernel_log())
    since = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    cmd = ['gst-launch-1.0', '-v', '-e']
    for i in ids:
        cmd += ['nvarguscamerasrc', f'sensor-id={i}', f'num-buffers={frames}', '!',
                'video/x-raw(memory:NVMM)', '!', 'fakesink', f'name=s{i}', 'silent=false',
                'sync=false']
    _, out, secs = run(cmd, timeout=40)
    clean_exit = 'Done Success' in out and 'Error generated' not in out
    locked = 'Failed to create CaptureSession' in out and secs < 3

    sensors = {}
    for i in ids:
        got = len(re.findall(rf'GstFakeSink:s{i}: last-message = chain', out))
        if got >= frames * 0.9:
            sensors[i] = ('PASS', f'{got} frames in {secs:.1f}s')
        elif got == 0 and clean_exit:
            # Frame lines missing but the pipeline finished cleanly.
            sensors[i] = ('PASS', f'pipeline finished cleanly in {secs:.1f}s '
                                  '(frame count not reported)')
        elif locked:
            sensors[i] = ('FAIL', 'Argus refused the capture session (camera locked or in '
                                  'use) even after a restart')
        else:
            sensors[i] = ('FAIL', f'no frames: {got}/{frames} received, gave up after '
                                  f'{secs:.0f}s')

    new = kernel_log()[before:]
    kernel = [ln for ln in new if KERNEL_CAPTURE_ERR.search(ln) and not KERNEL_BENIGN.search(ln)]
    _, journal, _ = run(['journalctl', '-u', 'nvargus-daemon', '--since', since, '--no-pager',
                         '-o', 'cat'], log_output=False)
    argus = []
    for ln in journal.splitlines():
        if re.search(r'\berror\b', ln, re.I) and not ARGUS_BENIGN.search(ln):
            short = re.sub(r'\s*\((in|propagating from) .*?\)', '', ln).strip()
            if short not in argus:
                argus.append(short)
    log('Kernel lines during test:\n' + '\n'.join(new) + '\n')
    log('Argus errors during test:\n' + '\n'.join(argus) + '\n')
    return sensors, kernel, argus, locked


def camera_status():
    restart_argus()
    code, out, _ = run(['ZED_Explorer', '-a'], timeout=60)
    if code == 127:
        return 'SKIP', 'ZED_Explorer not installed'
    cams = re.findall(r'Model :\s*"([^"]+)".*?S/N :\s*(\d+).*?State :\s*"([^"]+)"'
                      r'.*?Port :\s*(\d+)', out, re.S)
    if not cams:
        return 'FAIL', 'ZED_Explorer found no camera'
    details = '; '.join(f'{m} SN {sn} on port {p}: {st}' for m, sn, st, p in cams)
    return ('PASS' if all(c[2] == 'AVAILABLE' for c in cams) else 'FAIL'), details


def zed_sdk_test(res, fps, frames):
    user = os.environ.get('SUDO_USER')  # pyzed is installed per user, not for root
    restart_argus()
    _, out, secs = run([sys.executable, '-c', PYZED_TEST, res or 'AUTO', str(fps or 30),
                        str(frames)], timeout=180, user=user)
    if 'No module named' in out and 'pyzed' in out:
        return 'SKIP', f'pyzed is not installed for user {user or "root"}'
    m = re.search(r'^RESULT (\{.*\})$', out, re.M)
    if not m:
        return 'FAIL', f'test did not finish (stopped after {secs:.0f}s)'
    r = json.loads(m[1])
    if r['open'] != 'SUCCESS':
        errors = []
        for e in re.findall(r'\[ZED\]\[ERROR\]\s*(.+)', out):
            e = re.sub(r'\s+in sl::.*$', '', e).strip()
            if e not in errors:
                errors.append(e)
        return 'FAIL', f'open() = {r["open"]} after {r["open_s"]}s' + \
            (': ' + '; '.join(errors[:2]) if errors else '')
    rate = r['grabbed'] / r['grab_s'] if r['grab_s'] else 0
    status = 'PASS' if r['grabbed'] >= frames * 0.9 else 'FAIL'
    return status, (f'opened {r["model"]} SN {r["serial"]} at {r["width"]}x{r["height"]} @ '
                    f'{r["fps"]} fps; grabbed {r["grabbed"]}/{frames} stereo frames '
                    f'({rate:.1f} fps)')


# ----------------------------------------------------------------- report

def diagnose():
    """Turn the results into plain-language next steps."""
    r = {k: v.status for k, v in RESULTS.items()}
    notes = []
    if 'FAIL' in (r.get('dt'), r.get('modules')):
        notes.append('The ZED Link driver or its device tree overlay is not loaded. Reinstall '
                     'the stereolabs-zedlink package that matches this L4T version, then '
                     'reboot.')
    if 'FAIL' in (r.get('probe'), r.get('gmsl'), r.get('video')):
        notes.append('The kernel did not find the camera, or only one sensor, at boot. Power '
                     'off, reseat the GMSL cable at both ends, check camera power, then boot.')
    if r.get('services') == 'FAIL':
        down = RESULTS['services'].extra['down']
        if down:
            notes.append(f'Start the stopped services: sudo systemctl start {" ".join(down)}')
    if r.get('holders') == 'FAIL':
        notes.append('Another program is using the camera. Stop it (Ctrl+C the ROS launch, '
                     'close ZED tools) and run this again.')

    s = {i: RESULTS[f'argus{i}'] for i in (0, 1) if f'argus{i}' in RESULTS}
    kernel_errors = RESULTS.get('klog') and RESULTS['klog'].status == 'FAIL'
    if len(s) == 2:
        ok = [i for i in s if s[i].status == 'PASS']
        if all(v.extra.get('locked') for v in s.values()):
            notes.append('Argus refused every capture, even right after a restart. Reboot the '
                         'Jetson (do not run reset.sh) and run this again.')
        elif len(ok) == 1:
            bad = 1 - ok[0]
            where = ('Frames arrive damaged (the kernel logged capture errors): suspect the '
                     'GMSL cable or connectors.' if kernel_errors else
                     'The kernel logged no capture errors, so no data arrives at all: '
                     'suspect the camera or its connection.')
            notes.append(f'{sensor_label(bad).capitalize()} sends no frames while '
                         f'{sensor_label(ok[0])} works. The ZED SDK needs both, so the camera '
                         f'cannot open. {where} Power off and reseat the GMSL cable, try GMSL '
                         'port 1, swap the cable or camera, then contact Stereolabs. See '
                         'CAMERA_NOT_WORKING.md.')
        elif not ok:
            notes.append('Neither sensor sends frames. ' + (
                'The kernel logged capture errors: suspect the GMSL cable or ZED Link card.'
                if kernel_errors else 'Reboot and run this again; if it repeats, check the '
                                      'cable, then the ZED Link card.'))
        elif r.get('sdk') == 'FAIL':
            notes.append('Argus streams both sensors but the ZED SDK cannot open the camera. '
                         'Check that the SDK and driver versions match, and run '
                         'sudo ZED_Diagnostic -c.')

    if r.get('power') == 'WARN':
        notes.append('The power mode was changed after boot. If capture fails, reboot before '
                     'trusting these results.')
    if r.get('config') in ('WARN', 'FAIL'):
        notes.append(f'Check grab_resolution / grab_frame_rate: {RESULTS["config"].details}')
    if not notes and all(v in ('PASS', 'INFO', 'SKIP') for v in r.values()):
        notes.append('The camera is healthy. If jtop shows the GPU idle, object detection only '
                     'runs while something subscribes to /zedx/zed_node/obj_det/objects (see '
                     'CAMERA_NOT_WORKING.md).')
    return notes


def print_chart():
    width = shutil.get_terminal_size((100, 20)).columns
    rows = list(RESULTS.values())
    name_w = max(len(r.name) for r in rows)
    det_w = max(30, width - name_w - 14)
    rule = '─' * min(width, name_w + det_w + 14)
    print(f'\n{rule}\n {"#":>2}  {"Check":<{name_w}}  Result  Details\n{rule}')
    for n, r in enumerate(rows, 1):
        lines = textwrap.wrap(r.details, det_w) or ['']
        print(f' {n:>2}  {r.name:<{name_w}}  {badge(r.status)}    {lines[0]}')
        for line in lines[1:]:
            print(' ' * (name_w + 15) + line)
    print(rule)
    counts = {s: sum(r.status == s for r in rows) for s in STATUS_COLOR}
    print('  ' + ', '.join(color(f'{n} {s.lower()}', STATUS_COLOR[s])
                           for s, n in counts.items() if n))


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--skip-sdk', action='store_true',
                        help='skip the ZED SDK test (the slowest one, up to 2 min on failure)')
    parser.add_argument('--no-capture', action='store_true',
                        help='read-only checks only: no service restarts, no capture tests')
    parser.add_argument('--force', action='store_true',
                        help='run capture tests even if another program is using the camera')
    parser.add_argument('--frames', type=int, default=60,
                        help='frames to capture per test (default 60)')
    args = parser.parse_args()

    if os.geteuid() != 0:
        print('Needs root to restart the camera services; re-running with sudo.')
        os.execvp('sudo', ['sudo', sys.executable, os.path.abspath(__file__)] + sys.argv[1:])

    started = datetime.datetime.now()
    print(color(f'ZED X camera debug ({started:%Y-%m-%d %H:%M})', '1'))

    def check(key, name, func, *fargs):
        with Spinner(name):
            out = func(*fargs)
        record(key, name, out[0], out[1], **(out[2] if len(out) > 2 else {}))

    print(color('\nSystem', '36'))
    check('versions', 'Software versions', check_versions)
    check('power', 'Power mode', check_power)
    check('dt', 'Camera in device tree', check_device_tree)
    check('modules', 'Kernel driver modules', check_modules)
    with Spinner('Reading kernel log'):
        klog = kernel_log()
    check('probe', 'Kernel detects both sensors', check_sensor_probe, klog)
    check('gmsl', 'GMSL link', check_gmsl, klog)
    check('video', 'Video devices', check_video_devices)
    check('services', 'Camera services', check_services)
    check('holders', 'Another program using camera', check_holders)

    with Spinner('Sensor modes'):
        modes = sensor_modes()
    record('modes', 'Sensor modes (V4L2)', 'INFO' if modes else 'FAIL',
           fmt_modes(modes) or 'could not read modes from the zedx video device')
    res, fps = read_config()
    check('config', 'Configured resolution / fps', check_config, modes, res, fps)

    capture = not args.no_capture
    if capture and RESULTS['holders'].status == 'FAIL' and not args.force:
        print(color('\n  Skipping capture tests: another program is using the camera '
                    '(use --force to run them anyway).', '33'))
        capture = False

    if capture:
        print(color('\nCapture (nvargus-daemon is restarted before each test)', '36'))
        kernel, argus = [], []
        for ids in ([0], [1], [0, 1]):
            key = 'argus' + ''.join(map(str, ids)) if len(ids) == 1 else 'argus_both'
            name = (f'Argus: {sensor_label(ids[0])} alone' if len(ids) == 1
                    else 'Argus: both sensors together')
            with Spinner(name):
                sensors, k, a, locked = argus_capture(ids, args.frames)
            kernel += [ln for ln in k if ln not in kernel]
            argus += [ln for ln in a if ln not in argus]
            if len(ids) == 1:
                record(key, name, *sensors[ids[0]], locked=locked)
            else:
                failed = [i for i, v in sensors.items() if v[0] == 'FAIL']
                details = '; '.join(f'sensor {i}: {v[1]}' for i, v in sensors.items())
                record(key, name, 'FAIL' if failed else 'PASS', details, locked=locked)
        record('klog', 'Kernel log during sensor tests', 'FAIL' if kernel else 'PASS',
               ('capture errors: ' + ' | '.join(kernel[:3])) if kernel
               else 'no CSI / VI / deserializer errors logged')
        record('alog', 'Argus log during sensor tests', 'WARN' if argus else 'PASS',
               (f'{len(argus)} distinct errors, e.g. ' + ' | '.join(argus[:2])) if argus
               else 'no errors')

        check('status', 'Camera status (ZED_Explorer)', camera_status)
        if args.skip_sdk:
            record('sdk', 'ZED SDK alone (no ROS)', 'SKIP', '--skip-sdk')
        else:
            check('sdk', 'ZED SDK alone (no ROS)', zed_sdk_test, res, fps, args.frames)

        # A failed SDK open restarts zed_x_daemon, which stops IMU_Daemon and leaves it off.
        with Spinner('Cleaning up'):
            restart_argus()
            _, imu, _ = run(['systemctl', 'is-active', 'IMU_Daemon'])
            if imu.strip() != 'active':
                run(['systemctl', 'start', 'IMU_Daemon'])
        print('  Left nvargus-daemon freshly restarted' +
              (' and restarted IMU_Daemon.' if imu.strip() != 'active' else '.'))

    print_chart()
    notes = diagnose()
    if notes:
        print(color('\nWhat this means', '1'))
        for note in notes:
            print(textwrap.fill(note, shutil.get_terminal_size((100, 20)).columns - 2,
                                initial_indent='  - ', subsequent_indent='    '))

    log_path = f'/tmp/camera_debug_{started:%Y%m%d_%H%M%S}.log'
    with open(log_path, 'w') as f:
        f.write('\n'.join(LOG) + '\n')
    print(f'\nFull log (attach to a Stereolabs ticket): {log_path}')
    return 1 if any(r.status == 'FAIL' for r in RESULTS.values()) else 0


if __name__ == '__main__':
    sys.exit(main())
