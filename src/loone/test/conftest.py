import pytest
import rclpy


# Motor subclasses the real rclpy.node.Node (bound at import time via
# `from rclpy.node import Node`), so patching `loone.motor.rclpy` in the
# test fixtures never reaches it. Node.__init__ requires an initialized
# context, so we init rclpy once for the whole test session.
@pytest.fixture(scope='session', autouse=True)
def _ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
