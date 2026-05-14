filePath: /home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_bringup/launch/franka.launch.py

# 立即生效
sudo cpupower frequency-set -g performance

# 验证
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor  # 应输出 performance
如果不做修改 会报错  communication_constraints_violation （正常运行一会之后 由于 cpu的功耗管理 会报错）
以下是 `serl_franka_controllers` 在 **ROS2 Humble** 下的完整使用指南：

---

## ROS2 Humble 使用指南：`serl_franka_controllers`

### 1. 前置要求

- **ROS2 Humble** 已安装（`ros-humble-desktop` 或 `ros-humble-ros-base`）
- **`libfranka >= 0.9.0`** 和 **`franka_ros2`** 已从源码编译（位于你的 `ros2_ws/src/franka_ros2/`）
- **实时内核**（推荐，否则需在 `franka.config.yaml` 中设置 `realtime_config: ignore`，目前 ROS2 版 franka_ros2 暂未暴露该选项，需要确保 RT 内核）
- 机器人已通过网线连接到 PC，FCI 模式已激活（DESK 中激活，浏览器中操作）

### 2. 编译安装

```bash
# 进入你的 ROS2 工作空间
cd ~/hil-serl-ros2/ros2_ws

# 源码 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译 serl_franka_controllers
colcon build --packages-select serl_franka_controllers --symlink-install

# 加载环境
source install/setup.bash
```

---

### 3. 笛卡尔阻抗控制器 (Cartesian Impedance Controller)

这是 **最主要** 的控制器，提供一个柔顺但精确的笛卡尔空间阻抗控制器，适合在线强化学习。

#### 启动方式

```bash
source /opt/ros/humble/setup.bash
source ~/hil-serl-ros2/ros2_ws/install/setup.bash

ros2 launch serl_franka_controllers impedance.launch.py robot_ip:=10.42.0.163 arm_id:=fr3 load_gripper:=true

ros2 launch serl_franka_controllers impedance.launch.py \
    robot_ip:=10.42.0.163 \
    arm_id:=fr3 \
    load_gripper:=true

# joint stae controller 改写为 effort（力矩）PD 控制模式；完全绕过 Franka 内部 motion generator 。 这样不会有差值过程导致的 jerk极大的问题
# 不然会报错 joint_motion_generator_acceleration_discontinuity

source install/setup.bash
ros2 launch serl_franka_controllers joint.launch.py robot_ip:=10.42.0.163 arm_id:=fr3 load_gripper:=true
```

| 参数 | 说明 | 默认值 |
|---|---|---|
| `robot_ip` | 机器人控制器的 IP 地址 | `172.16.0.3` |
| `arm_id` | 机械臂型号 (`fr3` / `panda`) | `fr3` |
| `namespace` | 机器人命名空间（多机器人场景） | `""` |
| `load_gripper` | 是否加载 Franka 手爪 | `true` |
| `use_fake_hardware` | 使用模拟硬件（不需要实物机器人） | `false` |
| `urdf_file` | URDF 描述文件 | `fr3/fr3.urdf.xacro` |

**启动成功后可以看到的话题：**
- `/cartesian_impedance_controller/franka_jacobian` — 零空间雅可比矩阵（ZeroJacobian 自定义消息）
- `/cartesian_impedance_controller/equilibrium_pose` — 接收目标位姿命令（`geometry_msgs/PoseStamped`）
- `/franka_robot_state_broadcaster/franka_robot_state` — 机器人状态（`FrankaRobotState`）

#### 发送目标位姿命令（Python 代码示例）

```python
import rclpy
from rclpy.node import Node
import geometry_msgs.msg as geom_msg
from scipy.spatial.transform import Rotation as R
import numpy as np
import time

rclpy.init()
node = rclpy.create_node('franka_control_api')

eepub = node.create_publisher(
    geom_msg.PoseStamped,
    '/cartesian_impedance_controller/equilibrium_pose',
    10
)

# 发送目标位姿: xyz + 四元数
msg = geom_msg.PoseStamped()
msg.header.frame_id = "0"
msg.header.stamp = node.get_clock().now().to_msg()
msg.pose.position = geom_msg.Point(x=0.5, y=0.0, z=0.2)

# 使用 scipy 计算四元数
quat = R.from_euler('xyz', [np.pi, 0, np.pi/2]).as_quat()
msg.pose.orientation = geom_msg.Quaternion(
    x=quat[0], y=quat[1], z=quat[2], w=quat[3]
)
eepub.publish(msg)
```

---

### 4. 关节位置控制器 (Joint Position Controller)

用于**复位关节**到指定位置（原 ROS1 中用于长时间运行后的关节重置）。

#### 使用方式

在 ROS2 中，目标关节位置**不再通过 `rosparam` 设置**，而是通过启动时传入参数：

```bash
source ~/hil-serl-ros2/ros2_ws/install/setup.bash

ros2 launch serl_franka_controllers joint.launch.py \
    robot_ip:=172.16.0.2 \
    load_gripper:=true
```

控制器会平滑地从当前关节角度移动到配置文件中定义的目标角度（线性插值，耗时 10 秒）。

如需**自定义目标角度**，需要在启动前通过 ROS 2 参数方式设置：

```bash
# 方法 1：启动后通过命令行设置参数
ros2 param set /joint_position_controller target_joint_positions "[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"

# 方法 2：直接修改 config/serl_franka_controllers.yaml
```

---

### 5. FrankaServer（Flask API 服务）

`robot_servers/franka_server.py` 是一个 **Flask REST API 服务器**，为强化学习训练提供高层接口。

#### 启动方式

```bash
source ~/hil-serl-ros2/ros2_ws/install/setup.bash

cd ~/hil-serl-ros2/serl_robot_infra/robot_servers

python franka_server.py \
    --robot_ip=172.16.0.2 \
    --gripper_type=Franka \
    --flask_url=127.0.0.1
```

或者直接运行 shell 脚本：

```bash
bash launch_right_server.sh
```

#### 可用 API 端点

| 端点 | 方法 | 功能 |
|---|---|---|
| `/pose` | POST | 发送目标位姿 `[x,y,z,qx,qy,qz,qw]` |
| `/getpos` | POST | 获取当前末端位姿 |
| `/getvel` | POST | 获取当前末端速度 |
| `/getforce` | POST | 获取当前末端力 |
| `/gettorque` | POST | 获取当前末端力矩 |
| `/getq` | POST | 获取当前关节角度 |
| `/getdq` | POST | 获取当前关节速度 |
| `/getjacobian` | POST | 获取雅可比矩阵 |
| `/getstate` | POST | 一次性获取全部状态 |
| `/open_gripper` | POST | 打开手爪 |
| `/close_gripper` | POST | 关闭手爪 |
| `/clearerr` | POST | 清除错误（自动恢复） |
| `/jointreset` | POST | 重置关节（停止阻抗→复位→重启阻抗） |
| `/get_gripper` | POST | 获取手爪位置 |
| `/update_param` | POST | 在线更新控制参数（替代 ROS1 的 dynamic_reconfigure） |
| `/set_load` | POST | 设置末端负载参数 |

#### Python API 调用示例

```python
import requests
import numpy as np

url = "http://127.0.0.1:5000"

# 获取当前状态
r = requests.post(f"{url}/getstate")
state = r.json()
print(state["pose"], state["q"], state["gripper_pos"])

# 发送目标位姿
pose = [0.5, 0.0, 0.3, 0.0, 1.0, 0.0, 0.0]  # x,y,z,qx,qy,qz,qw
requests.post(f"{url}/pose", json={"arr": pose})

# 更新柔顺参数（替代 dynamic_reconfigure）
params = {
    "translational_stiffness_x": 2000.0,
    "translational_stiffness_y": 2000.0,
    "translational_stiffness_z": 2000.0,
}
requests.post(f"{url}/update_param", json=params)
```

---

### 6. 柔顺参数在线调整

> **关键变更：** ROS1 中使用 `dynamic_reconfigure` + `rqt_reconfigure` GUI 调整参数，在 ROS2 中改为通过参数服务。

#### 方法一：命令行

```bash
# 查看当前参数
ros2 param list /cartesian_impedance_controller

# 设置参数
ros2 param set /cartesian_impunity_controller nullspace_stiffness 25.0
```

#### 方法二：Python 代码

```python
import rclpy
from rclpy.parameter import Parameter

rclpy.init()
node = rclpy.create_node('param_client')

# 更新参数
param_cli = node.create_client(SetParameters, '/cartesian_impedance_controller/set_parameters')
# 通过 Flask 的 /update_param 端点更方便
```

#### 方法三：通过 Flask API

使用上面提到的 `/update_param` 端点（推荐，REPL 训练时更方便）。

---

### 7. 多机器人 / 命名空间场景

ROS2 原生支持命名空间，同一台 PC 可以控制多台 Franka 机器人：

```bash
# 机器人 A（右臂）
ros2 launch serl_franka_controllers impedance.launch.py \
    namespace:=right \
    robot_ip:=172.16.0.2

# 机器人 B（左臂）
ros2 launch serl_franka_controllers impedance.launch.py \
    namespace:=left \
    robot_ip:=173.16.0.2
```

对应 Flask 服务器也需使用不同端口：
```bash
python franka_server.py --robot_ip=172.16.0.2 --flask_url=0.0.0.0:5000
python franka_server.py --robot_ip=173.16.0.2 --flask_url=0.0.0.0:5001
```

---

### 8. ROS1 → ROS2 关键差异总结

| 操作 | ROS1 Noetic | ROS2 Humble |
|---|---|---|
| 启动控制器 | `roslaunch serl_franka_controllers impedance.launch` | `ros2 launch serl_franka_controllers impedance.launch.py` |
| 设置关节目标 | `rosparam set /target_joint_positions '[...]'` | `ros2 param set /joint_position_controller target_joint_positions "[...]"` |
| 调整柔顺参数 | `rqt_reconfigure` GUI 或 `dynamic_reconfigure.client` | `ros2 param set` 或 Flask `/update_param` |
| 命令格式 | `robot_ip:=172.16.0.2` | `robot_ip:=172.16.0.2`（语法相同） |
| 编译 | `catkin_make` | `colcon build --symlink-install` |
| 获取机器人状态 | `franka_msgs/FrankaState`（O_T_EE 为 float64[16]） | `franka_msgs/FrankaRobotState`（O_T_EE 为 PoseStamped） |
| 手爪控制 | 发布 `GraspActionGoal` 到 topic | 使用 `franka_msgs/action/Grasp` ActionClient |
| 错误恢复 | 发布 `ErrorRecoveryActionGoal` | 使用 `franka_msgs/action/ErrorRecovery` ActionClient |


### 3. ⚠️ Gripper 方案2 — 不修改外部包不可行

`FrankaHardwareInterface` 的 `command_interfaces_info_` 硬编码为 `kNumberOfJoints = 7` 个关节（[franka_hardware_interface.cpp:L79](file:///home/xusj/hil-serl-ros2/ros2_ws/src/franka_ros2/franka_hardware/src/franka_hardware_interface.cpp#L79)）。如果控制器同时声明 arm + gripper = 8 个 effort 接口，`prepare_command_mode_switch` 的校验会失败。需要修改 `franka_hardware`。

**推荐替代方案：** `franka_gripper` 的 position-force hybrid 模式已经可以实现类似阻抗的效果：

```bash
# 柔顺抓取（5N 微小力）
ros2 action send_goal /franka_gripper/grasp franka_gripper/GraspAction \
  "{width: 0.02, speed: 0.05, force: 5.0, epsilon: {inner: 0.005, outer: 0.005}}"
```

---

**下一步：先执行 `sudo cpupower frequency-set -g performance` 再启动测试。** 这应该能解决 `communication_constraints_violation` 的问题。