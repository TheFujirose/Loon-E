# Placeholder for Adafruit Blinka's `digitalio` module, used only so that
# `import digitalio` succeeds in CI (no real SBC hardware / GPIO present).
# hardware_controller.py only ever touches this module through calls that the
# test suite patches with MagicMock before they run, so no real behavior
# is needed here. Never shadows the real Blinka `digitalio` on robot hardware,
# since this directory is only added to PYTHONPATH for CI test runs.


class Direction:
    INPUT = object()
    OUTPUT = object()


class DigitalInOut:
    def __init__(self, *args, **kwargs):
        pass
