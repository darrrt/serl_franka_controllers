#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KeyboardTriggerNode(Node):

    def __init__(self):
        super().__init__('keyboard_trigger_node')

        self.teach_trigger_pub = self.create_publisher(
            Bool, '~/teach_trigger', 10)
        self.reset_pub = self.create_publisher(
            Bool, '~/reset', 10)

        self.teach_topic = self.declare_parameter(
            'teach_trigger_topic', '/data_collection_controller/teach_trigger'
        ).get_parameter_value().string_value
        self.reset_topic = self.declare_parameter(
            'reset_topic', '/data_collection_controller/reset'
        ).get_parameter_value().string_value

        self.teach_trigger_pub = self.create_publisher(
            Bool, self.teach_topic, 10)
        self.reset_pub = self.create_publisher(
            Bool, self.reset_topic, 10)

# 使用 Python 的 f-string 替代 C 风格的 %s 格式化
        self.get_logger().info(
            f'Keyboard trigger node started.\n'
            f'  [1] = confirm teach position (publish to {self.teach_topic})\n'
            f'  [r] = reset controller       (publish to {self.reset_topic})\n'
            f'  [q] = quit'
        )
        # self.get_logger().info(
        #     'Keyboard trigger node started.\n'
        #     '  [1] = confirm teach position (publish to %s)\n'
        #     '  [r] = reset controller       (publish to %s)\n'
        #     '  [q] = quit',
        #     self.teach_topic, self.reset_topic)

        self._old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self._run()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def _run(self):
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == '1':
                    msg = Bool()
                    msg.data = True
                    self.teach_trigger_pub.publish(msg)
                    self.get_logger().info('-> Teach trigger sent')
                elif key == 'r':
                    msg = Bool()
                    msg.data = True
                    self.reset_pub.publish(msg)
                    self.get_logger().info('-> Reset trigger sent')
                elif key == 'q':
                    self.get_logger().info('Quitting...')
                    break
            rclpy.spin_once(self, timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    try:
        KeyboardTriggerNode()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
