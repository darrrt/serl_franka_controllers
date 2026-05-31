#!/usr/bin/env python3
import csv
import json
import os
import shutil
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String


class DataRecorderNode(Node):

    def __init__(self):
        super().__init__('data_recorder_node')

        self.runs_dir = self.declare_parameter(
            'runs_dir', '/home/xusj/hil-serl-ros2/ros2_ws/runs'
        ).get_parameter_value().string_value

        self.controller_prefix = self.declare_parameter(
            'controller_prefix', '/data_collection_controller'
        ).get_parameter_value().string_value

        self.area_name = self.declare_parameter(
            'area_name', 'default'
        ).get_parameter_value().string_value

        self.calib_file = self.declare_parameter(
            'calib_file', ''
        ).get_parameter_value().string_value

        now = datetime.now().strftime('%Y-%m-%d_%H-%M')
        self.run_dir = os.path.join(self.runs_dir, now)
        os.makedirs(self.run_dir, exist_ok=True)
        self.get_logger().info(f'Run directory: {self.run_dir}')

        if self.calib_file and os.path.isfile(self.calib_file):
            try:
                with open(self.calib_file, 'r') as f:
                    calib_data = json.load(f)
                calib_dest = os.path.join(self.run_dir, 'calibration_result.json')
                with open(calib_dest, 'w') as f:
                    json.dump(calib_data, f, indent=2)
                self.get_logger().info(
                    f'Calibration file copied from {self.calib_file} to {calib_dest}')
            except Exception as e:
                self.get_logger().warn(f'Failed to read calib_file: {e}')

        self.rt_fields = [
            'timestamp_sec', 'timestamp_nsec',
            'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7',
            'dj1', 'dj2', 'dj3', 'dj4', 'dj5', 'dj6', 'dj7',
            'tau1', 'tau2', 'tau3', 'tau4', 'tau5', 'tau6', 'tau7',
            'F_ext_x', 'F_ext_y', 'F_ext_z',
            'Tau_ext_x', 'Tau_ext_y', 'Tau_ext_z',
            'cart_x', 'cart_y', 'cart_z',
            'orient_w', 'orient_x', 'orient_y', 'orient_z',
            'phase',
        ]

        self.phase_fields = ['timestamp_sec', 'phase']

        self.current_phase = 'UNKNOWN'
        self.last_phase = 'UNKNOWN'
        self.record_count = 0
        self.flush_interval = 500
        self.lock = threading.Lock()

        self.current_rt_file = None
        self.current_rt_writer = None
        self.area_count = 0

        self.phase_csv_path = os.path.join(self.run_dir, 'phase_log.csv')
        self.phase_csv_file = open(self.phase_csv_path, 'w', newline='')
        self.phase_writer = csv.DictWriter(self.phase_csv_file, fieldnames=self.phase_fields)
        self.phase_writer.writeheader()
        self.phase_csv_file.flush()

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        init_filename = f'{self.area_name}_init_{now_str}.csv'
        init_filepath = os.path.join(self.run_dir, init_filename)
        self.current_rt_file = open(init_filepath, 'w', newline='')
        self.current_rt_writer = csv.DictWriter(
            self.current_rt_file, fieldnames=self.rt_fields)
        self.current_rt_writer.writeheader()
        self.current_rt_file.flush()
        self.get_logger().info(f'Initial data file: {init_filename}')

        rt_topic = f'{self.controller_prefix}/rt_state'
        phase_topic = f'{self.controller_prefix}/phase'
        completion_topic = f'{self.controller_prefix}/completion'
        calib_result_topic = f'{self.controller_prefix}/calib_result'

        self.rt_sub = self.create_subscription(
            JointState, rt_topic, self.rt_callback, 10)
        self.phase_sub = self.create_subscription(
            String, phase_topic, self.phase_callback, 10)
        self.completion_sub = self.create_subscription(
            Bool, completion_topic, self.completion_callback, 10)
        self.calib_sub = self.create_subscription(
            Float64MultiArray, calib_result_topic, self.calib_callback, 10)

        self.get_logger().info(
            f'Data recorder started.\n'
            f'  Subscribing: {rt_topic}, {phase_topic}, {completion_topic}, {calib_result_topic}\n'
            f'  Recording to: {self.run_dir}\n'
            f'  Area name: {self.area_name}\n'
            f'  Calib file: {self.calib_file if self.calib_file else "(none)"}'
        )

    def _open_new_rt_file(self, area_label):
        if self.current_rt_file is not None:
            self.current_rt_file.flush()
            self.current_rt_file.close()

        now_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{self.area_name}_{area_label}_{now_str}.csv'
        filepath = os.path.join(self.run_dir, filename)

        self.current_rt_file = open(filepath, 'w', newline='')
        self.current_rt_writer = csv.DictWriter(
            self.current_rt_file, fieldnames=self.rt_fields)
        self.current_rt_writer.writeheader()
        self.current_rt_file.flush()

        self.get_logger().info(f'New data file: {filename}')

    def rt_callback(self, msg: JointState):
        with self.lock:
            if self.current_rt_writer is None:
                return

            n = len(msg.name)
            data = {}
            data['timestamp_sec'] = msg.header.stamp.sec
            data['timestamp_nsec'] = msg.header.stamp.nanosec

            for i in range(min(n, 7)):
                data[f'j{i+1}'] = msg.position[i] if i < len(msg.position) else 0.0
            for i in range(7, min(n, 14)):
                data[f'dj{i-6}'] = msg.position[i] if i < len(msg.position) else 0.0
            for i in range(14, min(n, 21)):
                data[f'tau{i-13}'] = msg.position[i] if i < len(msg.position) else 0.0

            for i in range(21, min(n, 27)):
                keys = ['F_ext_x', 'F_ext_y', 'F_ext_z',
                        'Tau_ext_x', 'Tau_ext_y', 'Tau_ext_z']
                if i - 21 < len(keys):
                    data[keys[i - 21]] = msg.position[i] if i < len(msg.position) else 0.0

            cart_keys = ['cart_x', 'cart_y', 'cart_z',
                         'orient_w', 'orient_x', 'orient_y', 'orient_z']
            for i in range(27, min(n, 34)):
                if i - 27 < len(cart_keys):
                    data[cart_keys[i - 27]] = msg.position[i] if i < len(msg.position) else 0.0

            if n > 34:
                data['phase'] = int(msg.position[34])
            else:
                data['phase'] = 0

            for field in self.rt_fields:
                if field not in data:
                    data[field] = 0.0

            self.current_rt_writer.writerow(data)
            self.record_count += 1

            if self.record_count % self.flush_interval == 0:
                self.current_rt_file.flush()

    def phase_callback(self, msg: String):
        old_phase = self.current_phase
        self.current_phase = msg.data

        if self.current_phase != self.last_phase:
            with self.lock:
                now = self.get_clock().now()
                self.phase_writer.writerow({
                    'timestamp_sec': now.nanoseconds / 1e9,
                    'phase': self.current_phase,
                })
                self.phase_csv_file.flush()
            self.last_phase = self.current_phase

        if self.current_phase == 'AREA_RISE' and old_phase != 'AREA_RISE':
            self.area_count += 1
            with self.lock:
                self._open_new_rt_file(f'area{self.area_count}')

    def completion_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Collection cycle completed!')
            completion_path = os.path.join(self.run_dir, 'completions.txt')
            with open(completion_path, 'a') as f:
                now = self.get_clock().now()
                f.write(f'{now.nanoseconds / 1e9}\n')

    def calib_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 4:
            calib_data = {
                'payload_mass_kg': msg.data[0],
                'payload_com': {
                    'x': msg.data[1],
                    'y': msg.data[2],
                    'z': msg.data[3],
                },
                'timestamp': datetime.now().isoformat(),
            }
            calib_path = os.path.join(self.run_dir, 'calibration_result.json')
            with open(calib_path, 'w') as f:
                json.dump(calib_data, f, indent=2)
            self.get_logger().info(
                f'Calibration result saved: mass={msg.data[0]:.4f} kg, '
                f'CoM=[{msg.data[1]:.4f}, {msg.data[2]:.4f}, {msg.data[3]:.4f}]')

    def destroy_node(self):
        with self.lock:
            if self.current_rt_file is not None:
                self.current_rt_file.flush()
                self.current_rt_file.close()
            self.phase_csv_file.flush()
            self.phase_csv_file.close()
        self.get_logger().info(
            f'Data recorder stopped. Total records: {self.record_count}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
