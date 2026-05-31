import json
import os
import re
import tempfile
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _inject_yaml_param(yaml_str, key, value):
    if isinstance(value, bool):
        yaml_val = 'true' if value else 'false'
    elif isinstance(value, float):
        yaml_val = str(value)
    elif isinstance(value, list):
        yaml_val = '[' + ', '.join(str(v) for v in value) + ']'
    elif isinstance(value, str):
        yaml_val = value
    else:
        yaml_val = str(value)

    pattern = re.compile(r'^(\s+' + re.escape(key) + r'\s*:\s*).+$', re.MULTILINE)
    match = pattern.search(yaml_str)
    if match:
        yaml_str = yaml_str[:match.start(1)] + match.group(1) + yaml_val + yaml_str[match.end():]
    return yaml_str


def generate_data_collection_nodes(context):
    load_gripper_launch_configuration = LaunchConfiguration('load_gripper').perform(context)
    load_gripper = load_gripper_launch_configuration.lower() == 'true'
    urdf_path = PathJoinSubstitution([
        FindPackageShare('franka_description'), 'robots',
        LaunchConfiguration('urdf_file')
    ]).perform(context)
    robot_description = xacro.process_file(
        urdf_path,
        mappings={
            'ros2_control': 'true',
            'arm_id': LaunchConfiguration('arm_id').perform(context),
            'arm_prefix': '',
            'robot_ip': LaunchConfiguration('robot_ip').perform(context),
            'hand': load_gripper_launch_configuration,
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
            'fake_sensor_commands': 'false',
        }
    ).toprettyxml(indent='  ')

    namespace = LaunchConfiguration('namespace').perform(context)

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('serl_franka_controllers'), 'config',
        'serl_franka_controllers.yaml'
    ]).perform(context)

    with open(controllers_yaml, 'r') as f:
        yaml_content = f.read()

    skip_grasp = LaunchConfiguration('skip_grasp').perform(context).lower() == 'true'
    skip_calibrate = LaunchConfiguration('skip_calibrate').perform(context).lower() == 'true'
    calib_file = LaunchConfiguration('calib_file').perform(context)

    yaml_content = _inject_yaml_param(yaml_content, 'skip_grasp', skip_grasp)
    yaml_content = _inject_yaml_param(yaml_content, 'skip_calibrate', skip_calibrate)

    if skip_calibrate and calib_file and os.path.isfile(calib_file):
        try:
            with open(calib_file, 'r') as f:
                calib_data = json.load(f)
            payload_mass = calib_data.get('payload_mass_kg', 0.0)
            payload_com = calib_data.get('payload_com', {})
            com_x = payload_com.get('x', 0.0)
            com_y = payload_com.get('y', 0.0)
            com_z = payload_com.get('z', 0.0)
            yaml_content = _inject_yaml_param(yaml_content, 'payload_mass', payload_mass)
            yaml_content = _inject_yaml_param(yaml_content, 'payload_com', [com_x, com_y, com_z])
            print(f'[launch] Loaded calibration from {calib_file}: '
                  f'mass={payload_mass:.4f} kg, CoM=[{com_x:.4f}, {com_y:.4f}, {com_z:.4f}]')
        except Exception as e:
            print(f'[launch] WARNING: Failed to read calib_file {calib_file}: {e}')

    temp_yaml_fd, temp_yaml_path = tempfile.mkstemp(suffix='.yaml', prefix='serl_franka_')
    with os.fdopen(temp_yaml_fd, 'w') as f:
        f.write(yaml_content)

    joint_state_publisher_sources = ['franka/joint_states', 'franka_gripper/joint_states']

    controller_prefix = f'/{namespace}/data_collection_controller' if namespace else '/data_collection_controller'

    area_name = LaunchConfiguration('area_name').perform(context)
    runs_dir = LaunchConfiguration('runs_dir').perform(context)
    collection_config = LaunchConfiguration('collection_config').perform(context)
    board_origin_str = LaunchConfiguration('board_origin').perform(context)

    board_origin = [0.0, 0.0, 0.0]
    if board_origin_str:
        try:
            board_origin = [float(x.strip()) for x in board_origin_str.split(',')]
            while len(board_origin) < 3:
                board_origin.append(0.0)
        except ValueError:
            pass

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            namespace=namespace,
            parameters=[
                temp_yaml_path,
                {'robot_description': robot_description},
                {'load_gripper': load_gripper}],
            remappings=[('joint_states', joint_state_publisher_sources[0])],
            output='screen',
            on_exit=Shutdown(),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[{
                'source_list': joint_state_publisher_sources,
                'rate': 30,
            }],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['franka_robot_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['data_collection_controller'],
            output='screen',
        ),
        Node(
            package='serl_franka_controllers',
            executable='data_recorder_node.py',
            name='data_recorder_node',
            namespace=namespace,
            parameters=[{
                'controller_prefix': controller_prefix,
                'runs_dir': runs_dir,
                'area_name': area_name,
                'calib_file': calib_file,
            }],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([PathJoinSubstitution(
                [FindPackageShare('franka_gripper'), 'launch', 'gripper.launch.py'])]),
            launch_arguments={
                'namespace': namespace,
                'robot_ip': LaunchConfiguration('robot_ip').perform(context),
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware').perform(context),
            }.items(),
            condition=IfCondition(LaunchConfiguration('load_gripper')),
        ),
    ]

    return nodes


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument('arm_id',
                              default_value='fr3',
                              description='ID of the type of arm used'),
        DeclareLaunchArgument('namespace',
                              default_value='',
                              description='Namespace for the robot'),
        DeclareLaunchArgument('urdf_file',
                              default_value='fr3/fr3.urdf.xacro',
                              description='Path to URDF file'),
        DeclareLaunchArgument('robot_ip',
                              default_value='10.42.0.163',
                              description='Hostname or IP address of the robot'),
        DeclareLaunchArgument('load_gripper',
                              default_value='true',
                              description='Use Franka Gripper as an end-effector'),
        DeclareLaunchArgument('use_fake_hardware',
                              default_value='false',
                              description='Use fake hardware'),
        DeclareLaunchArgument('runs_dir',
                              default_value='/home/xusj/hil-serl-ros2/ros2_ws/runs',
                              description='Directory for run data'),
        DeclareLaunchArgument('skip_grasp',
                              default_value='false',
                              description='Skip grasp phase, start at TEACH'),
        DeclareLaunchArgument('skip_calibrate',
                              default_value='false',
                              description='Skip calibration phase, go directly to WAIT_PARAMS after TEACH trigger'),
        DeclareLaunchArgument('calib_file',
                              default_value='',
                              description='Path to calibration result JSON file (used when skip_calibrate=true)'),
        DeclareLaunchArgument('area_name',
                              default_value='default',
                              description='Name prefix for recorded data files'),
        DeclareLaunchArgument('collection_config',
                              default_value='',
                              description='Path to collection config JSON file (e.g. calibration board definition)'),
        DeclareLaunchArgument('board_origin',
                              default_value='0.0,0.0,0.0',
                              description='Board origin position in robot base frame as x,y,z (meters)'),
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=generate_data_collection_nodes)])
