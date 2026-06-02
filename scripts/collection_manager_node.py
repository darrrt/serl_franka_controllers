#!/usr/bin/env python3
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


class CollectionManagerNode(Node):

    PHASE_WAIT_PARAMS = 7

    def __init__(self):
        super().__init__('collection_manager_node')

        self.controller_prefix = self.declare_parameter(
            'controller_prefix', '/data_collection_controller'
        ).get_parameter_value().string_value

        self.collection_config = self.declare_parameter(
            'collection_config', ''
        ).get_parameter_value().string_value

        self.default_f0 = self.declare_parameter(
            'default_f0', 1.0
        ).get_parameter_value().double_value

        self.default_v0 = self.declare_parameter(
            'default_v0', 0.01
        ).get_parameter_value().double_value

        self.params_topic = f'{self.controller_prefix}/collection_params'
        self.phase_topic = f'{self.controller_prefix}/phase'
        self.completion_topic = f'{self.controller_prefix}/completion'
        self.teach_trigger_topic = f'{self.controller_prefix}/teach_trigger'
        self.reset_topic = f'{self.controller_prefix}/reset'
        self.task_name_topic = f'{self.controller_prefix}/task_name'

        self.params_pub = self.create_publisher(
            Float64MultiArray, self.params_topic, 10)
        self.teach_trigger_pub = self.create_publisher(
            Bool, self.teach_trigger_topic, 10)
        self.reset_pub = self.create_publisher(
            Bool, self.reset_topic, 10)
        self.task_name_pub = self.create_publisher(
            String, self.task_name_topic, 10)
        self.phase_sub = self.create_subscription(
            String, self.phase_topic, self.phase_callback, 10)
        self.completion_sub = self.create_subscription(
            Bool, self.completion_topic, self.completion_callback, 10)

        self.current_phase = 'UNKNOWN'
        self.is_waiting_params = False
        self.cycle_complete = False
        self.cycle_count = 0

        self.task_queue = []
        self.current_task = None
        self.task_names = []

        self.get_logger().info(
            f'Collection manager started (v4 protocol).\n'
            f'  Publishing to: {self.params_topic}, {self.teach_trigger_topic}, {self.reset_topic}\n'
            f'  Monitoring: {self.phase_topic}, {self.completion_topic}\n'
            f'  Collection config: {self.collection_config if self.collection_config else "(none)"}\n'
            f'  Protocol: [board_x, board_y, F_0, v_0, switch_phase]\n'
            f'  (coordinates are in calibration board frame, controller converts to robot frame)\n'
            f'  Commands:\n'
            f'    [1] = confirm teach position (TEACH -> next phase)\n'
            f'    [a] = add default task\n'
            f'    [s] = add area-switch task\n'
            f'    [l] = load tasks from config file\n'
            f'    [r] = run all queued tasks\n'
            f'    [c] = clear task queue\n'
            f'    [R] = reset controller (shift+r)\n'
            f'    [p] = print status\n'
            f'    [q] = quit'
        )

    def phase_callback(self, msg: String):
        old_phase = self.current_phase
        self.current_phase = msg.data

        if self.current_phase != old_phase:
            self.get_logger().info(f'Phase: {old_phase} -> {self.current_phase}')

        if self.current_phase == 'WAIT_PARAMS' and old_phase != 'WAIT_PARAMS':
            self.is_waiting_params = True
            self.get_logger().info('Robot is now waiting for params.')
            self._try_send_next()

    def completion_callback(self, msg: Bool):
        if msg.data:
            self.cycle_complete = True
            self.cycle_count += 1
            task_name = self.task_names[self.cycle_count - 1] if self.cycle_count <= len(self.task_names) else '?'
            self.get_logger().info(
                f'Cycle #{self.cycle_count} ({task_name}) complete! '
                f'Queue remaining: {len(self.task_queue)}')

    def _try_send_next(self):
        if not self.is_waiting_params:
            return

        if not self.task_queue:
            self.get_logger().info('No more tasks in queue.')
            return

        self.current_task = self.task_queue.pop(0)

        task = self.current_task
        task_name = task.get('name', 'unnamed')

        task_name_msg = String()
        task_name_msg.data = task_name
        self.task_name_pub.publish(task_name_msg)

        msg = Float64MultiArray()
        msg.data = [
            task['board_x'],
            task['board_y'],
            task['F0'],
            task['v0'],
            1.0 if task.get('switch_phase', False) else 0.0,
        ]

        self.params_pub.publish(msg)
        self.is_waiting_params = False
        self.cycle_complete = False

        self.get_logger().info(
            f'Sent task [{task_name}]: board=[{task["board_x"]:.4f}, {task["board_y"]:.4f}] '
            f'F0={task["F0"]:.2f} v0={task["v0"]:.4f} '
            f'switch_phase={task.get("switch_phase", False)}')

    def add_task(self, board_x=0.0, board_y=0.0, F0=1.0, v0=0.01,
                 switch_phase=False, name='manual'):
        task = {
            'name': name,
            'board_x': board_x,
            'board_y': board_y,
            'F0': F0,
            'v0': v0,
            'switch_phase': switch_phase,
        }
        self.task_queue.append(task)
        self.task_names.append(name)
        self.get_logger().info(
            f'Task added [{name}] (queue size: {len(self.task_queue)}): '
            f'board=[{board_x:.4f}, {board_y:.4f}] '
            f'F0={F0:.2f} v0={v0:.4f} switch_phase={switch_phase}')

    def load_tasks_from_config(self, config_path=None):
        if config_path is None:
            config_path = self.collection_config

        if not config_path:
            self.get_logger().warn('No collection config file specified!')
            return

        if not os.path.isfile(config_path):
            self.get_logger().warn(f'Config file not found: {config_path}')
            return

        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'Config file is not valid JSON: {config_path}\n'
                f'  JSON error: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Failed to read config file: {e}')
            return

        if 'calibration_board' in config:
            self._load_calibration_board(config['calibration_board'])
        elif 'tasks' in config:
            self._load_tasks_list(config['tasks'])
        else:
            self.get_logger().warn(
                'Config file has no "calibration_board" or "tasks" key')

    def _load_calibration_board(self, board_config):
        points = board_config.get('points_3d', {}).get('data', [])

        if not points:
            self.get_logger().warn('No points_3d data in calibration_board config')
            return

        self.get_logger().info(
            f'Loading calibration board: {len(points)} cells (board frame)')

        prev_row = None
        for i, point in enumerate(points):
            name = point.get('name', f'cell_{i}')
            pos_mm = point.get('pos_mm', [0.0, 0.0, 0.0])
            row = point.get('row', 0)

            board_x = pos_mm[0] / 1000.0
            board_y = pos_mm[1] / 1000.0

            switch_phase = (prev_row is not None and row != prev_row) or (i == 0)

            task = {
                'name': name,
                'board_x': board_x,
                'board_y': board_y,
                'F0': self.default_f0,
                'v0': self.default_v0,
                'switch_phase': switch_phase,
            }
            self.task_queue.append(task)
            self.task_names.append(name)
            prev_row = row

        self.get_logger().info(
            f'Loaded {len(points)} tasks from calibration board config. '
            f'Queue size: {len(self.task_queue)}')

    def _load_tasks_list(self, tasks):
        for i, task_data in enumerate(tasks):
            name = task_data.get('name', f'task_{i}')
            task = {
                'name': name,
                'board_x': task_data.get('board_x', 0.0),
                'board_y': task_data.get('board_y', 0.0),
                'F0': task_data.get('F0', self.default_f0),
                'v0': task_data.get('v0', self.default_v0),
                'switch_phase': task_data.get('switch_phase', False),
            }
            self.task_queue.append(task)
            self.task_names.append(name)

        self.get_logger().info(
            f'Loaded {len(tasks)} tasks from tasks list. '
            f'Queue size: {len(self.task_queue)}')

    def clear_queue(self):
        count = len(self.task_queue)
        self.task_queue.clear()
        self.task_names.clear()
        self.get_logger().info(f'Cleared {count} tasks from queue')

    def run_queued_tasks(self):
        if not self.task_queue:
            self.get_logger().warn('No tasks in queue!')
            return

        self.get_logger().info(
            f'Running {len(self.task_queue)} queued tasks...')

        if self.is_waiting_params:
            self._try_send_next()

    def print_status(self):
        task_list = ''
        for i, name in enumerate(self.task_names):
            marker = ' -> ' if i == self.cycle_count else '    '
            task_list += f'\n{marker}[{i}] {name}'
        if not task_list:
            task_list = '\n    (empty)'

        self.get_logger().info(
            f'Status: phase={self.current_phase} '
            f'waiting_params={self.is_waiting_params} '
            f'cycle_complete={self.cycle_complete} '
            f'cycle_count={self.cycle_count} '
            f'queue_size={len(self.task_queue)}'
            f'\nTasks:{task_list}')


def main(args=None):
    rclpy.init(args=args)
    node = CollectionManagerNode()

    import select
    import sys
    import termios
    import tty

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key == '1':
                    msg = Bool()
                    msg.data = True
                    node.teach_trigger_pub.publish(msg)
                    node.get_logger().info('-> Teach trigger sent')
                elif key == 'a':
                    node.add_task(
                        board_x=0.032, board_y=0.0,
                        F0=node.default_f0, v0=node.default_v0)
                elif key == 's':
                    node.add_task(
                        board_x=0.032, board_y=0.0,
                        F0=node.default_f0, v0=node.default_v0,
                        switch_phase=True, name='area_switch')
                elif key == 'l':
                    node.load_tasks_from_config()
                elif key == 'r':
                    node.run_queued_tasks()
                elif key == 'c':
                    node.clear_queue()
                elif key == 'R':
                    msg = Bool()
                    msg.data = True
                    node.reset_pub.publish(msg)
                    node.get_logger().info('-> Reset trigger sent')
                elif key == 'p':
                    node.print_status()
                elif key == 'q':
                    node.get_logger().info('Quitting...')
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
