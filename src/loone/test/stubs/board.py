# Placeholder for Adafruit Blinka's `board` module, used only so that
# `import board` succeeds in CI (no real SBC hardware present).
# motor.py only ever touches this module through calls that the test
# suite patches with MagicMock before they run, so no real behavior
# is needed here. Never shadows the real Blinka `board` on robot hardware,
# since this directory is only added to PYTHONPATH for CI test runs.

SCL = object()
SDA = object()
