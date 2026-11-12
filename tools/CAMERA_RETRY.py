#!/usr/bin/env python3
"""Reset the camera stack over and over until sensor 1 streams.

Sensor 1 (zedx_right_0@20, the right image sensor) is the one that sends no
frames (see CAMERA_NOT_WORKING.md). This cycles through heavier and heavier
resets and, after each one, captures from sensor 1 alone through Argus. It stops
at the first capture that gets frames, or on Ctrl+C.

Resets, in order, repeated:
  argus   restart nvargus-daemon
  rebind  unbind and rebind sensor 1's kernel driver (re-runs its probe)
  full    stop IMU_Daemon / zed_x_daemon / nvargus-daemon, clear the Argus
          socket and IPC shared memory, reload the ZED kernel modules, start
          everything again (what tools/fast_reset does)

Needs root; started as a normal user, it re-runs itself with sudo.

    tools/CAMERA_RETRY.py                    # until it works or Ctrl+C
    tools/CAMERA_RETRY.py --max-tries 20
    tools/CAMERA_RETRY.py --resets argus,full

Stop the ROS launch first, or the capture tests will fight it for the camera.
"""

import argparse
import datetime
import glob
import itertools
import os
import sys
import time

sys.dont_write_bytecode = True  # running as root; don't leave a root-owned __pycache__
import CAMERA_DEBUG as dbg  # noqa: E402

SENSOR_NODE = 'zedx_right_0@20'
ZEDX_DRIVER = '/sys/bus/i2c/drivers/zedx'


def find_sensor():
    """Return (Argus sensor-id, I2C device such as '10-0020') for SENSOR_NODE."""
    dbg.check_device_tree()
    sid = next((i for i, n in dbg.SENSOR_NAMES.items() if n == SENSOR_NODE), None)
    dev = next((os.path.basename(p) for p in glob.glob('/sys/bus/i2c/devices/*')
                if os.path.basename(os.path.realpath(os.path.join(p, 'of_node'))) == SENSOR_NODE),
               None)
    return sid, dev


def systemctl(*args, timeout=60):
    return dbg.run(['systemctl', *args], timeout=timeout)[0]


def active(svc):
    return systemctl('is-active', '--quiet', svc) == 0


def wait_active(svc, secs):
    end = time.monotonic() + secs
    while not active(svc):
        if time.monotonic() > end:
            return False
        time.sleep(0.5)
    return True


def modules_missing():
    loaded = {ln.split()[0] for ln in dbg.read_dt('/proc/modules').splitlines() if ln}
    return [m for m in dbg.MODULES if m not in loaded]


def bound(dev):
    return os.path.exists(os.path.join(ZEDX_DRIVER, dev))


def sysfs_write(path, value):
    """Write to a sysfs file; return the error message, or None on success."""
    dbg.log(f'echo {value} > {path}')
    try:
        with open(path, 'w') as f:
            f.write(value)
    except OSError as e:
        return e.strerror
    return None


# ----------------------------------------------------------------- resets
# Each takes sensor 1's I2C device and returns (status, details).

def reset_argus(dev):
    # argus_capture() restarts nvargus-daemon before it captures, so the test
    # itself is the reset.
    return 'PASS', 'nvargus-daemon is restarted by the capture test'


def reset_rebind(dev):
    systemctl('stop', 'nvargus-daemon')  # Argus holds /dev/video*
    if bound(dev):
        sysfs_write(os.path.join(ZEDX_DRIVER, 'unbind'), dev)
        time.sleep(1)
    err = sysfs_write(os.path.join(ZEDX_DRIVER, 'bind'), dev)
    time.sleep(1)
    if not bound(dev):
        return 'FAIL', f'zedx driver did not bind to {dev}' + (f' ({err})' if err else '')
    return 'PASS', f'zedx driver re-probed {dev}'


def reset_full(dev):
    # Dependents first: IMU_Daemon -> zed_x_daemon -> nvargus-daemon.
    for svc in reversed(dbg.SERVICES):
        systemctl('stop', svc)
    dbg.run(['rm', '-rf', '/tmp/argus_socket'])
    dbg.run(['ipcrm', '-a'])
    systemctl('start', 'nvargus-daemon')
    wait_active('nvargus-daemon', 5)
    # ZEDX_Driver rmmods and re-insmods the ZED modules; it has to run before
    # zed_x_daemon starts.
    systemctl('restart', 'driver_zed_loader', timeout=120)
    problems = ['driver_zed_loader failed'] \
        if systemctl('is-failed', '--quiet', 'driver_zed_loader') == 0 else []
    for svc, secs in (('zed_x_daemon', 8), ('IMU_Daemon', 3)):
        systemctl('start', svc)
        wait_active(svc, secs)
    problems += [f'{m} not loaded' for m in modules_missing()]
    if not bound(dev):
        problems.append(f'{dev} not probed')
    problems += [f'{s} not running' for s in dbg.SERVICES if not active(s)]
    if problems:
        return 'FAIL', 'stack restarted but ' + ', '.join(problems)
    return 'PASS', 'services restarted, Argus socket and IPC cleared, ZED driver reloaded'


RESETS = {
    'argus': ('Restart Argus', reset_argus),
    'rebind': ('Rebind sensor driver', reset_rebind),
    'full': ('Full reset + driver reload', reset_full),
}


def capture(sid, frames):
    sensors, kernel, argus, locked = dbg.argus_capture([sid], frames)
    status, details = sensors[sid]
    if status == 'FAIL':
        if kernel:
            details += f'; kernel: {kernel[0]}'
        if argus:
            details += f'; {len(argus)} Argus errors, e.g. {argus[0]}'
    return status, details


def restore(dev):
    """Put the stack back together, whatever state a reset was cut off in."""
    if dev and not bound(dev):
        sysfs_write(os.path.join(ZEDX_DRIVER, 'bind'), dev)
    if modules_missing():
        systemctl('restart', 'driver_zed_loader', timeout=120)
    dbg.restart_argus()  # a capture killed by Ctrl+C leaves Argus locked
    started = []
    for svc in dbg.SERVICES[1:]:
        if not active(svc):
            systemctl('start', svc)
            started.append(svc)
    return started


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--resets', default=','.join(RESETS),
                        help=f'comma-separated resets to cycle through, in order '
                             f'(default {",".join(RESETS)})')
    parser.add_argument('--frames', type=int, default=30,
                        help='frames to capture per test (default 30)')
    parser.add_argument('--max-tries', type=int, default=0,
                        help='give up after this many tries (default 0: never)')
    parser.add_argument('--force', action='store_true',
                        help='run even if another program is using the camera')
    args = parser.parse_args()
    order = [r.strip() for r in args.resets.split(',') if r.strip()]
    unknown = [r for r in order if r not in RESETS]
    if unknown or not order:
        parser.error(f'unknown reset(s): {", ".join(unknown) or "none given"} '
                     f'(choose from {", ".join(RESETS)})')

    if os.geteuid() != 0:
        print('Needs root to restart the camera services; re-running with sudo.')
        os.execvp('sudo', ['sudo', sys.executable, os.path.abspath(__file__)] + sys.argv[1:])

    started = datetime.datetime.now()
    print(dbg.color(f'ZED X sensor 1 retry ({started:%Y-%m-%d %H:%M})', '1'))

    sid, dev = find_sensor()
    if sid is None:
        print(dbg.color(f'  {SENSOR_NODE} is not a camera module in the device tree.', '31'))
        return 1
    if dev is None and 'rebind' in order:
        print(dbg.color(f'  No I2C device for {SENSOR_NODE}; skipping the rebind reset.', '33'))
        order = [r for r in order if r != 'rebind'] or ['argus']
    status, details, _ = dbg.check_holders()
    if status == 'FAIL' and not args.force:
        print(dbg.color(f'  {details}. Stop it first, or use --force.', '31'))
        return 1

    print(f'  Testing {dbg.sensor_label(sid)}' + (f', I2C {dev}' if dev else '') +
          f'; resets: {" -> ".join(order)}. Ctrl+C to stop.')
    tries = {r: 0 for r in order}
    success = None
    code = 1
    try:
        for n in itertools.count(1):
            if args.max_tries and n > args.max_tries:
                break
            key = order[(n - 1) % len(order)]
            name, func = RESETS[key]
            tries[key] += 1
            elapsed = datetime.datetime.now() - started
            print(dbg.color(f'\nTry {n}: {name}  ({str(elapsed).split(".")[0]} elapsed)', '36'))
            with dbg.Spinner(name):
                status, details = func(dev)
            dbg.record(f'reset{n}', name, status, details)
            label = f'Argus: {dbg.sensor_label(sid)} alone'
            with dbg.Spinner(label):
                status, details = capture(sid, args.frames)
            dbg.record(f'capture{n}', label, status, details)
            if status == 'PASS':
                success = (n, key)
                code = 0
                break
    except KeyboardInterrupt:
        print(dbg.color('\n  Stopped (Ctrl+C).', '33'))
        code = 130

    with dbg.Spinner('Restoring camera services'):
        restarted = restore(dev)
    print('  Left nvargus-daemon freshly restarted' +
          (f' and started {", ".join(restarted)}.' if restarted else '.'))

    elapsed = str(datetime.datetime.now() - started).split('.')[0]
    total = sum(tries.values())
    counts = ', '.join(f'{RESETS[r][0].lower()} x{c}' for r, c in tries.items() if c)
    print()
    if success:
        print(dbg.color(f'Sensor 1 streamed on try {success[0]} after '
                        f'"{RESETS[success[1]][0].lower()}" ({elapsed}).', '32'))
    else:
        print(dbg.color(f'Sensor 1 never streamed: {total} tries in {elapsed}.', '31'))
    if counts:
        print(f'  Resets run: {counts}')

    log_path = f'/tmp/camera_retry_{started:%Y%m%d_%H%M%S}.log'
    with open(log_path, 'w') as f:
        f.write('\n'.join(dbg.LOG) + '\n')
    print(f'Full log: {log_path}')
    return code


if __name__ == '__main__':
    sys.exit(main())
