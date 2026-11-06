"""task: the boat's task logic. A shell for future development.

TODO: There are no tasks right now, so this node does nothing. Until it exists,
terminal_node relays commands typed by hand. When this is written, it should:

    * Publish motor_command and ball_launcher_command as terminal_node does (see
      hardware_node for their formats).
    * Fail closed on rc_state as terminal_node does, since hardware_node obeys any
      command it is sent, including while the e-stop is engaged.
    * Be added to launch/launch.py, with condition=UnlessCondition(sim).
"""

import rclpy
from rclpy.node import Node


class TaskNode(Node):
    """Decide what the boat does next. Not implemented yet."""

    def __init__(self) -> None:
        super().__init__('task')
        self.get_logger().warning('Task logic is not implemented yet; doing nothing.')


def main(args=None) -> None:
    """Initialize the ROS2 node and spin."""
    rclpy.init(args=args)
    node = TaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('task interrupted by user.')
    finally:
        node.destroy_node()
        # The SIGINT handler may have already shut down the context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
