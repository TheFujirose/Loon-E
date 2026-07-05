# Placeholder for Adafruit Blinka's `busio` module, used only so that
# `import busio` succeeds in CI (no real SBC hardware / I2C bus present).
# motor.py only ever touches this module through calls that the test
# suite patches with MagicMock before they run, so no real behavior
# is needed here. Never shadows the real Blinka `busio` on robot hardware,
# since this directory is only added to PYTHONPATH for CI test runs.


class I2C:
    def __init__(self, *args, **kwargs):
        pass
