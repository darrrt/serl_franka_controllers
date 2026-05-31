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

        self.board_origin = self.declare_parameter(
            'board_origin', [0.0, 0.0, 0.0]
        ).get_parameter_value().double_array_value

        self.default_dx = self.declare_parameter(
            'default_dx', 0.01
        ).get_parameter_value().double_value

        self.default_dy = self.declare_parameter(
            'default_dy', 0.0
        ).get_parameter_value().double_value

        self.default_dx_step = self.declare_parameter(
            'default_dx_step', 0.01
        ).get_parameter_value().double_value

        self.default_dy_step = self.declare_parameter(
            'default_dy_step', 0.0
        ).get_parameter_value().double_value

        self.default_f0 = self.declare_parameter(
            'default_f0', 1.0
        ).get_parameter_value().double_value

        self.params_topic = f'{self.controller_prefix}/collection_params'
        self.phase_topic = f'{self.controller_prefix}/phase'
        self.completion_topic = f'{self.controller_prefix}/completion'
        self.teach_trigger_topic = f'{self.controller_prefix}/teach_trigger'
        self.reset_topic = f'{self.controller_prefix}/reset'

        self.params_pub = self.create_publisher(
            Float64MultiArray, self.params_topic, 10)
        self.teach_trigger_pub = self.create_publisher(
            Bool, self.teach_trigger_topic, 10)
        self.reset_pub = self.create_publisher(
            Bool, self.reset_topic, 10)
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
            f'Collection manager started.\n'
            f'  Publishing to: {self.params_topic}, {self.teach_trigger_topic}, {self.reset_topic}\n'
            f'  Monitoring: {self.phase_topic}, {self.completion_topic}\n'
            f'  Collection config: {self.collection_config if self.collection_config else "(none)"}\n'
            f'  Board origin: [{self.board_origin[0]:.4f}, {self.board_origin[1]:.4f}, {self.board_origin[2]:.4f}]\n'
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
        msg = Float64MultiArray()
        msg.data = [
            task['dx'], task['dy'],
            task['dx_step'], task['dy_step'],
            task['F0'],
            1.0 if task.get('switch_area', False) else 0.0,
            task.get('target_x', 0.0),
            task.get('target_y', 0.0),
        ]

        self.params_pub.publish(msg)
        self.is_waiting_params = False
        self.cycle_complete = False

        task_name = task.get('name', 'unnamed')
        self.get_logger().info(
            f'Sent task [{task_name}]: dx={task["dx"]:.4f} dy={task["dy"]:.4f} '
            f'dx_step={task["dx_step"]:.4f} dy_step={task["dy_step"]:.4f} '
            f'F0={task["F0"]:.2f} '
            f'switch_area={task.get("switch_area", False)} '
            f'target=[{task.get("target_x", 0):.4f}, {task.get("target_y", 0):.4f}]')

    def add_task(self, dx=0.01, dy=0.0, dx_step=0.01, dy_step=0.0, F0=1.0,
                 switch_area=False, target_x=0.0, target_y=0.0, name='manual'):
        task = {
            'name': name,
            'dx': dx, 'dy': dy,
            'dx_step': dx_step, 'dy_step': dy_step,
            'F0': F0,
            'switch_area': switch_area,
            'target_x': target_x, 'target_y': target_y,
        }
        self.task_queue.append(task)
        self.task_names.append(name)
        self.get_logger().info(
            f'Task added [{name}] (queue size: {len(self.task_queue)}): '
            f'dx={dx:.4f} dy={dy:.4f} dx_step={dx_step:.4f} dy_step={dy_step:.4f} '
            f'F0={F0:.2f} switch_area={switch_area}')

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
                f'  JSON error: {e}\n'
                f'  Hint: collection_config expects a JSON file (e.g. calibration_sandpaper_240-1500.json), not a YAML file')
            return
        except Exception as e:
            self.get_logger().error(f'Failed to read config file: {e}')
            return

        origin_x = float(self.board_origin[0])
        origin_y = float(self.board_origin[1])

        if 'calibration_board' in config:
            self._load_calibration_board(config['calibration_board'], origin_x, origin_y)
        elif 'tasks' in config:
            self._load_tasks_list(config['tasks'], origin_x, origin_y)
        else:
            self.get_logger().warn(
                'Config file has no "calibration_board" or "tasks" key')

    def _load_calibration_board(self, board_config, origin_x, origin_y):
        layout = board_config.get('layout', {})
        points = board_config.get('points_3d', {}).get('data', [])

        if not points:
            self.get_logger().warn('No points_3d data in calibration_board config')
            return

        grid_size_x = layout.get('grid_size_x', 30.0)
        grid_size_y = layout.get('grid_size_y', 30.0)

        self.get_logger().info(
            f'Loading calibration board: {len(points)} cells, '
            f'grid_size=({grid_size_x}, {grid_size_y})mm, '
            f'origin=({origin_x:.4f}, {origin_y:.4f})m')

        for i, point in enumerate(points):
            name = point.get('name', f'cell_{i}')
            pos_mm = point.get('pos_mm', [0.0, 0.0, 0.0])

            target_x = origin_x + pos_mm[0] / 1000.0
            target_y = origin_y + pos_mm[1] / 1000.0

            switch_area = (i > 0)

            task = {
                'name': name,
                'dx': self.default_dx,
                'dy': self.default_dy,
                'dx_step': self.default_dx_step,
                'dy_step': self.default_dy_step,
                'F0': self.default_f0,
                'switch_area': switch_area,
                'target_x': target_x,
                'target_y': target_y,
            }
            self.task_queue.append(task)
            self.task_names.append(name)

        self.get_logger().info(
            f'Loaded {len(points)} tasks from calibration board config. '
            f'Queue size: {len(self.task_queue)}')

    def _load_tasks_list(self, tasks, origin_x, origin_y):
        for i, task_data in enumerate(tasks):
            name = task_data.get('name', f'task_{i}')
            task = {
                'name': name,
                'dx': task_data.get('dx', self.default_dx),
                'dy': task_data.get('dy', self.default_dy),
                'dx_step': task_data.get('dx_step', self.default_dx_step),
                'dy_step': task_data.get('dy_step', self.default_dy_step),
                'F0': task_data.get('F0', self.default_f0),
                'switch_area': task_data.get('switch_area', False),
                'target_x': task_data.get('target_x', 0.0) + origin_x,
                'target_y': task_data.get('target_y', 0.0) + origin_y,
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
                        dx=node.default_dx, dy=node.default_dy,
                        dx_step=node.default_dx_step, dy_step=node.default_dy_step,
                        F0=node.default_f0)
                elif key == 's':
                    node.add_task(
                        dx=node.default_dx, dy=node.default_dy,
                        dx_step=node.default_dx_step, dy_step=node.default_dy_step,
                        F0=node.default_f0,
                        switch_area=True, target_x=0.3, target_y=0.0,
                        name='area_switch')
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
