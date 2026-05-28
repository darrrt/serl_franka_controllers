#include <serl_franka_controllers/joint_force_slide_controller.h>

#include <bitset>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <franka/model.h>
#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka/robot_state.h>

namespace serl_franka_controllers {

/**
 * @brief 配置命令接口
 * 
 * 控制器需要的命令接口：7个关节的力矩控制接口
 * @return 接口配置对象
 */
controller_interface::InterfaceConfiguration
JointForceSlideController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

/**
 * @brief 配置状态接口
 * 
 * 控制器需要的状态接口：
 * - 7个关节位置
 * - 7个关节速度
 * - Franka机器人模型提供的状态接口（包含雅可比矩阵、科里奥利力等）
 * @return 接口配置对象
 */
controller_interface::InterfaceConfiguration
JointForceSlideController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (const auto& name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(name);
  }
  return config;
}

/**
 * @brief 控制器初始化函数
 * 
 * 声明所有参数的默认值，包括：
 * - 机械臂标识和目标关节位置
 * - 关节空间控制器增益 (k_gains, d_gains)
 * - 力控滑动相关参数（目标力、滑动距离、下降速度等）
 * - 阻抗控制刚度和阻尼参数（不同阶段使用不同参数）
 * - 安全限制参数（最大关节速度、最大笛卡尔速度、最大力矩）
 * @return 初始化成功返回 SUCCESS，失败返回 ERROR
 */
CallbackReturn JointForceSlideController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("target_joint_positions", {});
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
    auto_declare<double>("motion_duration", 10.0);
    auto_declare<std::string>("start_phase", "joint_move");

    auto_declare<double>("target_force", 1.0);
    auto_declare<double>("slide_distance", 0.1);
    auto_declare<double>("descend_speed", 0.005);
    auto_declare<double>("slide_speed", 0.01);
    auto_declare<double>("force_kp", 0.005);
    auto_declare<double>("force_ki", 0.01);
    auto_declare<double>("force_filter_alpha", 0.1);
    auto_declare<double>("max_descend_distance", 0.15);

    auto_declare<double>("translational_stiffness_xy", 2000.0);
    auto_declare<double>("translational_stiffness_z_settle", 2000.0);
    auto_declare<double>("translational_stiffness_z_descend", 1500.0);
    auto_declare<double>("translational_stiffness_z_slide", 200.0);
    auto_declare<double>("rotational_stiffness", 150.0);
    auto_declare<double>("translational_damping_xy", 89.0);
    auto_declare<double>("translational_damping_z_settle", 89.0);
    auto_declare<double>("translational_damping_z_descend", 60.0);
    auto_declare<double>("translational_damping_z_slide", 30.0);
    auto_declare<double>("rotational_damping", 7.0);
    auto_declare<double>("nullspace_stiffness", 20.0);
    auto_declare<double>("joint1_nullspace_stiffness", 20.0);

    auto_declare<double>("max_joint_velocity", 2.0);
    auto_declare<double>("max_cartesian_velocity", 0.5);
    auto_declare<double>("max_torque", 30.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 控制器配置函数
 * 
 * 从ROS参数服务器获取所有参数并初始化：
 * 1. 创建Franka机器人模型实例
 * 2. 加载目标关节位置和控制器增益
 * 3. 加载力控滑动参数
 * 4. 加载阻抗控制刚度和阻尼参数
 * 5. 创建各类发布器（相位、力、诊断信息等）
 * 6. 创建复位订阅器
 * @param previous_state 之前的生命周期状态
 * @return 配置成功返回 SUCCESS，失败返回 ERROR
 */
CallbackReturn JointForceSlideController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state");

  auto target_positions = get_node()->get_parameter("target_joint_positions").as_double_array();
  if (target_positions.size() != static_cast<size_t>(num_joints_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointForceSlideController: target_joint_positions size is %zu, expected %d",
                 target_positions.size(), num_joints_);
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < target_positions.size(); ++i) {
    target_joint_pose_[i] = target_positions[i];
  }

  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  if (k.size() == 7 && d.size() == 7) {
    for (size_t i = 0; i < 7; ++i) {
      k_gains_[i] = k[i];
      d_gains_[i] = d[i];
    }
  }

  motion_duration_ = get_node()->get_parameter("motion_duration").as_double();
  start_phase_str_ = get_node()->get_parameter("start_phase").as_string();
  target_force_ = get_node()->get_parameter("target_force").as_double();
  slide_distance_ = get_node()->get_parameter("slide_distance").as_double();
  descend_speed_ = get_node()->get_parameter("descend_speed").as_double();
  slide_speed_ = get_node()->get_parameter("slide_speed").as_double();
  force_kp_ = get_node()->get_parameter("force_kp").as_double();
  force_ki_ = get_node()->get_parameter("force_ki").as_double();
  force_filter_alpha_ = get_node()->get_parameter("force_filter_alpha").as_double();
  max_descend_distance_ = get_node()->get_parameter("max_descend_distance").as_double();

  translational_stiffness_xy_ = get_node()->get_parameter("translational_stiffness_xy").as_double();
  translational_stiffness_z_settle_ = get_node()->get_parameter("translational_stiffness_z_settle").as_double();
  translational_stiffness_z_descend_ = get_node()->get_parameter("translational_stiffness_z_descend").as_double();
  translational_stiffness_z_slide_ = get_node()->get_parameter("translational_stiffness_z_slide").as_double();
  rotational_stiffness_ = get_node()->get_parameter("rotational_stiffness").as_double();
  translational_damping_xy_ = get_node()->get_parameter("translational_damping_xy").as_double();
  translational_damping_z_settle_ = get_node()->get_parameter("translational_damping_z_settle").as_double();
  translational_damping_z_descend_ = get_node()->get_parameter("translational_damping_z_descend").as_double();
  translational_damping_z_slide_ = get_node()->get_parameter("translational_damping_z_slide").as_double();
  rotational_damping_ = get_node()->get_parameter("rotational_damping").as_double();
  nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
  nullspace_stiffness_target_ = nullspace_stiffness_;
  joint1_nullspace_stiffness_ = get_node()->get_parameter("joint1_nullspace_stiffness").as_double();
  joint1_nullspace_stiffness_target_ = joint1_nullspace_stiffness_;

  max_joint_velocity_ = get_node()->get_parameter("max_joint_velocity").as_double();
  max_cartesian_velocity_ = get_node()->get_parameter("max_cartesian_velocity").as_double();
  max_torque_ = get_node()->get_parameter("max_torque").as_double();

  RCLCPP_INFO(get_node()->get_logger(),
              "Configured: start_phase='%s', K_z_settle=%.0f, K_z_descend=%.0f, K_z_slide=%.0f, descend_speed=%.4f",
              start_phase_str_.c_str(),
              translational_stiffness_z_settle_,
              translational_stiffness_z_descend_,
              translational_stiffness_z_slide_,
              descend_speed_);

  phase_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
      "~/phase", rclcpp::SystemDefaultsQoS());
  completion_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/completion", rclcpp::SystemDefaultsQoS());
  filtered_force_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
      "~/filtered_force_z", rclcpp::SystemDefaultsQoS());
  diagnostics_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "~/diagnostics", rclcpp::SystemDefaultsQoS());
  timing_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
      "~/update_time_us", rclcpp::SystemDefaultsQoS());
  command_torques_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "~/command_torques", rclcpp::SystemDefaultsQoS());
  force_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/force_z", rclcpp::SystemDefaultsQoS());

  reset_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/reset", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { resetCallback(msg); });

  return CallbackReturn::SUCCESS;
}

/**
 * @brief 将字符串转换为相位枚举
 * 
 * @param phase_str 相位字符串（settle/descend/slide）
 * @return 对应的相位枚举值
 */
JointForceSlideController::Phase JointForceSlideController::parsePhaseString(
    const std::string& phase_str) {
  if (phase_str == "settle" || phase_str == "SETTLE") return Phase::SETTLE;
  if (phase_str == "descend" || phase_str == "DESCEND") return Phase::DESCEND;
  if (phase_str == "slide" || phase_str == "SLIDE") return Phase::SLIDE;
  return Phase::JOINT_MOVE;
}

/**
 * @brief 将相位枚举转换为字符串
 * 
 * @param p 相位枚举值
 * @return 对应的字符串
 */
std::string JointForceSlideController::phaseToString(Phase p) {
  switch (p) {
    case Phase::JOINT_MOVE: return "JOINT_MOVE";
    case Phase::SETTLE: return "SETTLE";
    case Phase::DESCEND: return "DESCEND";
    case Phase::SLIDE: return "SLIDE";
    default: return "UNKNOWN";
  }
}

/**
 * @brief 根据当前相位设置笛卡尔刚度和阻尼矩阵
 * 
 * 不同阶段使用不同的刚度和阻尼参数：
 * - SETTLE阶段：高刚度，确保稳定定位
 * - DESCEND阶段：中等刚度，允许一定柔性
 * - SLIDE阶段：低刚度，便于力控滑动
 * 
 * @param phase 当前相位
 */
void JointForceSlideController::setStiffnessForPhase(Phase phase) {
  double kz, dz;
  switch (phase) {
    case Phase::SETTLE:
      kz = translational_stiffness_z_settle_;
      dz = translational_damping_z_settle_;
      break;
    case Phase::DESCEND:
      kz = translational_stiffness_z_descend_;
      dz = translational_damping_z_descend_;
      break;
    case Phase::SLIDE:
      kz = translational_stiffness_z_slide_;
      dz = translational_damping_z_slide_;
      break;
    default:
      kz = translational_stiffness_z_settle_;
      dz = translational_damping_z_settle_;
      break;
  }

  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3) =
      translational_stiffness_xy_ * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_(2, 2) = kz;
  cartesian_stiffness_target_.bottomRightCorner(3, 3) =
      rotational_stiffness_ * Eigen::Matrix3d::Identity();

  cartesian_damping_target_.setIdentity();
  cartesian_damping_target_.topLeftCorner(3, 3) =
      translational_damping_xy_ * Eigen::Matrix3d::Identity();
  cartesian_damping_target_(2, 2) = dz;
  cartesian_damping_target_.bottomRightCorner(3, 3) =
      rotational_damping_ * Eigen::Matrix3d::Identity();

  RCLCPP_INFO(get_node()->get_logger(),
              "setStiffnessForPhase(%s): K_z=%.0f, D_z=%.0f",
              phaseToString(phase).c_str(), kz, dz);
}

/**
 * @brief 初始化阻抗控制状态
 * 
 * 获取当前末端执行器的位姿作为阻抗控制的目标位姿，
 * 初始化零空间目标关节位置，并设置当前相位对应的刚度和阻尼参数。
 */
void JointForceSlideController::initImpedanceState() {
  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
  position_d_ = transform.translation();
  orientation_d_ = Eigen::Quaterniond(transform.linear());
  position_d_target_ = position_d_;
  orientation_d_target_ = orientation_d_;

  RCLCPP_INFO(get_node()->get_logger(),
              "initImpedanceState: pos_d=[%.4f, %.4f, %.4f]",
              position_d_.x(), position_d_.y(), position_d_.z());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_current(q_.data());
  q_d_nullspace_ = q_current;
  error_i_.setZero();

  setStiffnessForPhase(phase_);

  cartesian_stiffness_ = cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
}

/**
 * @brief 控制器激活函数
 * 
 * 激活控制器时执行以下操作：
 * 1. 为Franka机器人模型分配状态接口
 * 2. 更新关节状态
 * 3. 获取机器人状态指针
 * 4. 保存初始关节位置
 * 5. 根据配置解析起始相位
 * 6. 如果不是从JOINT_MOVE开始，则初始化阻抗控制状态
 * 7. 根据起始相位初始化相关变量（力基线、起始位置等）
 * 8. 重置所有状态变量
 * @param previous_state 之前的生命周期状态
 * @return 激活成功返回 SUCCESS
 */
CallbackReturn JointForceSlideController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  updateJointStates();

  std::string full_interface_name = arm_id_ + "/robot_state";
  robot_state_interface_index_ = -1;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (state_interfaces_[i].get_name() == full_interface_name) {
      robot_state_interface_index_ = static_cast<int>(i);
      break;
    }
  }
  robot_state_ptr_ = getRobotStatePtr();

  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    initial_joint_pose_[i] = state_interfaces_[i].get_value();
  }
  joint_elapsed_time_ = rclcpp::Duration(0, 0);

  phase_ = parsePhaseString(start_phase_str_);

  RCLCPP_INFO(get_node()->get_logger(),
              "on_activate: start_phase='%s' -> phase=%s, robot_state_ptr=%p",
              start_phase_str_.c_str(), phaseToString(phase_).c_str(),
              static_cast<void*>(robot_state_ptr_));

  if (phase_ != Phase::JOINT_MOVE) {
    initImpedanceState();
    RCLCPP_INFO(get_node()->get_logger(),
                "Starting directly at phase %s (skipping JOINT_MOVE)",
                start_phase_str_.c_str());
  }

  if (phase_ == Phase::SETTLE) {
    settle_count_ = 0;
    filtered_force_z_ = 0.0;
  } else if (phase_ == Phase::DESCEND) {
    settle_count_ = settle_cycles_;
    baseline_force_z_ = 0.0;
    if (robot_state_ptr_) {
      double raw_force_z = robot_state_ptr_->O_F_ext_hat_K[2];
      for (int i = 0; i < 100; ++i) {
        filtered_force_z_ = force_filter_alpha_ * raw_force_z +
                            (1.0 - force_filter_alpha_) * filtered_force_z_;
      }
    }
    baseline_force_z_ = filtered_force_z_;

    auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
    descend_start_z_ = transform.translation().z();

    RCLCPP_INFO(get_node()->get_logger(),
                "Starting at DESCEND (baseline_force_z=%.3f, start_z=%.4f)",
                baseline_force_z_, descend_start_z_);
  } else if (phase_ == Phase::SLIDE) {
    settle_count_ = settle_cycles_;
    baseline_force_z_ = 0.0;
    filtered_force_z_ = 0.0;

    auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
    slide_start_x_ = transform.translation().x();
    slide_base_z_ = position_d_.z();
    position_filter_ = 0.1;

    RCLCPP_INFO(get_node()->get_logger(), "Starting at SLIDE (start_x=%.4f, base_z=%.4f)", slide_start_x_, slide_base_z_);
  }

  filtered_force_z_ = 0.0;
  force_error_integral_ = 0.0;
  z_offset_ = 0.0;
  safety_violation_ = false;
  max_update_time_us_ = 0.0;
  last_update_time_us_ = 0.0;
  update_count_ = 0;
  tau_J_d_last_.fill(0.0);
  last_cmd_tau_.fill(0.0);

  return CallbackReturn::SUCCESS;
}

/**
 * @brief 控制器停用函数
 * 
 * 停用控制器时释放资源：
 * 1. 释放Franka机器人模型的接口
 * 2. 重置机器人状态指针
 * @param previous_state 之前的生命周期状态
 * @return 停用成功返回 SUCCESS
 */
CallbackReturn JointForceSlideController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  robot_state_ptr_ = nullptr;
  robot_state_interface_index_ = -1;
  return CallbackReturn::SUCCESS;
}

/**
 * @brief 更新关节状态
 * 
 * 从状态接口读取当前关节位置和速度
 */
void JointForceSlideController::updateJointStates() {
  for (int i = 0; i < num_joints_; ++i) {
    q_.at(i) = state_interfaces_[i].get_value();
    dq_.at(i) = state_interfaces_[num_joints_ + i].get_value();
  }
}

/**
 * @brief 获取机器人状态指针
 * 
 * 从状态接口中提取Franka机器人状态结构体指针，
 * 该指针包含机器人的完整状态信息（外力、关节状态等）
 * @return 机器人状态指针，失败返回nullptr
 */
franka::RobotState* JointForceSlideController::getRobotStatePtr() {
  if (robot_state_interface_index_ < 0) {
    return nullptr;
  }
  double val = state_interfaces_[robot_state_interface_index_].get_value();
  franka::RobotState* ptr = nullptr;
  std::memcpy(&ptr, &val, sizeof(ptr));
  return ptr;
}

/**
 * @brief 饱和力矩变化率
 * 
 * 限制力矩指令的变化率，防止力矩突变导致机器人振动或损坏
 * @param tau_d_calculated 计算得到的目标力矩
 * @param tau_J_d 上一周期的力矩指令
 * @return 饱和后的力矩指令
 */
Eigen::Matrix<double, 7, 1> JointForceSlideController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

/**
 * @brief 计算阻抗控制力矩
 * 
 * 实现笛卡尔空间阻抗控制，包含以下步骤：
 * 1. 获取雅可比矩阵和科里奥利力向量
 * 2. 获取当前末端执行器位姿
 * 3. 计算笛卡尔空间误差（位置误差+姿态误差）
 * 4. 计算任务空间力矩（基于刚度和阻尼）
 * 5. 计算零空间力矩（保持关节在期望位置附近）
 * 6. 合成总力矩（任务力矩+零空间力矩+科里奥利力补偿）
 * 7. 饱和力矩变化率
 * 8. 平滑更新目标位置和控制器参数
 * 
 * @param tau_d 输出的力矩指令
 */
void JointForceSlideController::computeImpedanceControl(
    Eigen::Matrix<double, 7, 1>& tau_d) {
  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_.data());

  if (update_count_ < 5 || (update_count_ % 500 == 0 && (phase_ == Phase::SETTLE || phase_ == Phase::DESCEND || phase_ == Phase::SLIDE))) {
    double jac_norm = jacobian.norm();
    double coriolis_norm = coriolis.norm();

    std::array<double, 42> body_jac_array =
        franka_robot_model_->getBodyJacobian(franka::Frame::kEndEffector);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> body_jacobian(body_jac_array.data());

    std::array<double, 49> mass_array = franka_robot_model_->getMassMatrix();
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass_matrix(mass_array.data());

    std::array<double, 7> gravity_array = franka_robot_model_->getGravityForceVector();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity_vec(gravity_array.data());

    RCLCPP_INFO(get_node()->get_logger(),
                "[DIAG-DEEP] zero_jac_norm=%.4f body_jac_norm=%.4f mass_norm=%.4f grav_norm=%.4f coriolis_norm=%.4f",
                jac_norm, body_jacobian.norm(), mass_matrix.norm(), gravity_vec.norm(), coriolis_norm);

    RCLCPP_INFO(get_node()->get_logger(),
                "[DIAG-DEEP] zero_jac_raw[0:6]=[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]",
                jacobian_array[0], jacobian_array[1], jacobian_array[2],
                jacobian_array[3], jacobian_array[4], jacobian_array[5]);

    if (robot_state_ptr_) {
      RCLCPP_INFO(get_node()->get_logger(),
                  "[DIAG-DEEP] robot_state_q=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
                  robot_state_ptr_->q[0], robot_state_ptr_->q[1], robot_state_ptr_->q[2],
                  robot_state_ptr_->q[3], robot_state_ptr_->q[4], robot_state_ptr_->q[5],
                  robot_state_ptr_->q[6]);
      RCLCPP_INFO(get_node()->get_logger(),
                  "[DIAG-DEEP] F_T_EE=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
                  robot_state_ptr_->F_T_EE[0], robot_state_ptr_->F_T_EE[1],
                  robot_state_ptr_->F_T_EE[2], robot_state_ptr_->F_T_EE[3],
                  robot_state_ptr_->F_T_EE[4], robot_state_ptr_->F_T_EE[5],
                  robot_state_ptr_->F_T_EE[6], robot_state_ptr_->F_T_EE[7]);
      RCLCPP_INFO(get_node()->get_logger(),
                  "[DIAG-DEEP] EE_T_K=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]",
                  robot_state_ptr_->EE_T_K[0], robot_state_ptr_->EE_T_K[1],
                  robot_state_ptr_->EE_T_K[2], robot_state_ptr_->EE_T_K[3],
                  robot_state_ptr_->EE_T_K[4], robot_state_ptr_->EE_T_K[5],
                  robot_state_ptr_->EE_T_K[6], robot_state_ptr_->EE_T_K[7]);
      RCLCPP_INFO(get_node()->get_logger(),
                  "[DIAG-DEEP] dq=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] m_total=%.4f",
                  robot_state_ptr_->dq[0], robot_state_ptr_->dq[1], robot_state_ptr_->dq[2],
                  robot_state_ptr_->dq[3], robot_state_ptr_->dq[4], robot_state_ptr_->dq[5],
                  robot_state_ptr_->dq[6], robot_state_ptr_->m_total);

      std::array<double, 42> direct_jac_array = {0};
      if (robot_state_ptr_) {
        int robot_model_interface_index = -1;
        std::string model_interface_name = arm_id_ + "/robot_model";
        for (size_t i = 0; i < state_interfaces_.size(); ++i) {
          if (state_interfaces_[i].get_name() == model_interface_name) {
            robot_model_interface_index = static_cast<int>(i);
            break;
          }
        }
        if (robot_model_interface_index >= 0) {
          double model_val = state_interfaces_[robot_model_interface_index].get_value();
          franka_hardware::Model* hw_model_ptr = nullptr;
          std::memcpy(&hw_model_ptr, &model_val, sizeof(hw_model_ptr));
          RCLCPP_INFO(get_node()->get_logger(),
                      "[DIAG-BYPASS] hw_model_ptr=%p, vtable_ptr=%p",
                      static_cast<void*>(hw_model_ptr),
                      hw_model_ptr ? *reinterpret_cast<void**>(hw_model_ptr) : nullptr);

          if (hw_model_ptr && robot_state_ptr_) {
            direct_jac_array = hw_model_ptr->zeroJacobian(
                franka::Frame::kEndEffector,
                robot_state_ptr_->q,
                robot_state_ptr_->F_T_EE,
                robot_state_ptr_->EE_T_K);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> direct_jacobian(direct_jac_array.data());
            RCLCPP_INFO(get_node()->get_logger(),
                        "[DIAG-BYPASS] direct_zeroJacobian norm=%.4f (non-virtual call)",
                        direct_jacobian.norm());
          }
        }
      }
    }

    RCLCPP_INFO(get_node()->get_logger(),
                "[DIAG] K_diag=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f]",
                cartesian_stiffness_(0,0), cartesian_stiffness_(1,1), cartesian_stiffness_(2,2),
                cartesian_stiffness_(3,3), cartesian_stiffness_(4,4), cartesian_stiffness_(5,5));
    RCLCPP_INFO(get_node()->get_logger(),
                "[DIAG] K_target_diag=[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f] pos_filter=%.4f",
                cartesian_stiffness_target_(0,0), cartesian_stiffness_target_(1,1), cartesian_stiffness_target_(2,2),
                cartesian_stiffness_target_(3,3), cartesian_stiffness_target_(4,4), cartesian_stiffness_target_(5,5),
                position_filter_);
  }

  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  error_.head(3) << position - position_d_;
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error_.tail(3) << -transform.linear() * error_.tail(3);

  if (update_count_ < 5 || (update_count_ % 500 == 0 && (phase_ == Phase::SETTLE || phase_ == Phase::DESCEND || phase_ == Phase::SLIDE))) {
    RCLCPP_INFO(get_node()->get_logger(),
                "[DIAG] err=[%.5f,%.5f,%.5f,%.5f,%.5f,%.5f] pos=[%.4f,%.4f,%.4f] pos_d=[%.4f,%.4f,%.4f] pos_d_target=[%.4f,%.4f,%.4f]",
                error_(0), error_(1), error_(2), error_(3), error_(4), error_(5),
                position.x(), position.y(), position.z(),
                position_d_.x(), position_d_.y(), position_d_.z(),
                position_d_target_.x(), position_d_target_.y(), position_d_target_.z());
  }

  Eigen::Matrix<double, 6, 7> jacobian_transpose_pinv;
  Eigen::Matrix<double, 7, 6> jacobian_T = jacobian.transpose();
  pseudoInverse(jacobian_T, jacobian_transpose_pinv);

  Eigen::Matrix<double, 6, 1> cartesian_force =
      -cartesian_stiffness_ * error_ - cartesian_damping_ * (jacobian * dq);
  Eigen::Matrix<double, 7, 1> tau_task = jacobian.transpose() * cartesian_force;

  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Matrix<double, 7, 1> qe;
  qe << q_d_nullspace_ - q;
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe << dq;
  dqe.head(1) << dqe.head(1) * 2.0 * std::sqrt(joint1_nullspace_stiffness_);

  Eigen::Matrix<double, 7, 1> tau_nullspace =
      (Eigen::Matrix<double, 7, 7>::Identity() - jacobian.transpose() * jacobian_transpose_pinv) *
      (nullspace_stiffness_ * qe - (2.0 * std::sqrt(nullspace_stiffness_)) * dqe);

  tau_d = tau_task + tau_nullspace + coriolis;

  if (update_count_ < 5 || (update_count_ % 500 == 0 && (phase_ == Phase::SETTLE || phase_ == Phase::DESCEND || phase_ == Phase::SLIDE))) {
    double cart_force_norm = cartesian_force.norm();
    double tau_task_norm = tau_task.norm();
    double tau_null_norm = tau_nullspace.norm();
    double tau_d_pre_norm = tau_d.norm();
    RCLCPP_INFO(get_node()->get_logger(),
                "[DIAG] F_cart_norm=%.4f tau_task_norm=%.4f tau_null_norm=%.4f coriolis_norm=%.4f tau_d_pre_sat_norm=%.4f",
                cart_force_norm, tau_task_norm, tau_null_norm, coriolis.norm(), tau_d_pre_norm);
  }

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d_last(tau_J_d_last_.data());
  tau_d = saturateTorqueRate(tau_d, tau_J_d_last);

  if (update_count_ < 5 || (update_count_ % 500 == 0 && (phase_ == Phase::SETTLE || phase_ == Phase::DESCEND || phase_ == Phase::SLIDE))) {
    RCLCPP_INFO(get_node()->get_logger(),
                "[DIAG] tau_d_post_sat=[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f] tau_d_norm=%.4f",
                tau_d(0), tau_d(1), tau_d(2), tau_d(3), tau_d(4), tau_d(5), tau_d(6), tau_d.norm());
  }

  for (size_t i = 0; i < 7; ++i) {
    tau_J_d_last_[i] = tau_d(i);
  }

  position_d_ = position_filter_ * position_d_target_ + (1.0 - position_filter_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ =
      filter_params_ * joint1_nullspace_stiffness_target_ +
      (1.0 - filter_params_) * joint1_nullspace_stiffness_;
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
}

/**
 * @brief 检查安全限制
 * 
 * 检查关节速度和力矩是否超过安全限制，超过时发出警告
 * @return 如果有任何安全限制被违反返回true，否则返回false
 */
bool JointForceSlideController::checkSafetyLimits() {
  bool violation = false;

  for (int i = 0; i < num_joints_; ++i) {
    if (std::abs(dq_[i]) > max_joint_velocity_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Joint %d velocity %.3f exceeds limit %.3f",
                           i + 1, std::abs(dq_[i]), max_joint_velocity_);
      violation = true;
    }
    if (std::abs(tau_J_d_last_[i]) > max_torque_) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                           "Joint %d torque %.3f exceeds limit %.3f",
                           i + 1, std::abs(tau_J_d_last_[i]), max_torque_);
      violation = true;
    }
  }

  return violation;
}

/**
 * @brief 复位回调函数
 * 
 * 接收到复位消息后，重置控制器状态到JOINT_MOVE阶段
 */
void JointForceSlideController::resetCallback(const std_msgs::msg::Bool::SharedPtr /*msg*/) {
  phase_ = Phase::JOINT_MOVE;
  settle_count_ = 0;
  filtered_force_z_ = 0.0;
  baseline_force_z_ = 0.0;
  force_error_integral_ = 0.0;
  z_offset_ = 0.0;
  safety_violation_ = false;
  max_update_time_us_ = 0.0;
  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    initial_joint_pose_[i] = q_[i];
  }
  joint_elapsed_time_ = rclcpp::Duration(0, 0);
  RCLCPP_INFO(get_node()->get_logger(), "Controller reset -> JOINT_MOVE");
}

/**
 * @brief 控制器主更新函数
 * 
 * 每个控制周期执行一次，包含以下步骤：
 * 1. 更新关节状态
 * 2. 滤波外力数据
 * 3. 如果发生安全违规，输出零力矩并返回
 * 4. 根据当前相位执行相应的更新逻辑
 * 5. 检查安全限制
 * 6. 发布诊断信息和状态数据
 * 
 * @param time 当前时间
 * @param period 控制周期
 * @return 执行成功返回OK
 */
controller_interface::return_type JointForceSlideController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  auto t0 = std::chrono::steady_clock::now();

  updateJointStates();

  if (robot_state_ptr_) {
    double raw_force_z = robot_state_ptr_->O_F_ext_hat_K[2];
    filtered_force_z_ = force_filter_alpha_ * raw_force_z +
                        (1.0 - force_filter_alpha_) * filtered_force_z_;
  }

  if (safety_violation_) {
    for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
      command_interfaces_[i].set_value(0.0);
    }
    return controller_interface::return_type::OK;
  }

  switch (phase_) {
    case Phase::JOINT_MOVE:
      updateJointMove(period);
      break;
    case Phase::SETTLE:
      updateSettle();
      break;
    case Phase::DESCEND:
      updateDescend();
      break;
    case Phase::SLIDE:
      updateSlide();
      break;
  }

  if (checkSafetyLimits()) {
    safety_violation_ = true;
    RCLCPP_ERROR(get_node()->get_logger(), "Safety limit violated! Controller stopped.");
  }

  auto t1 = std::chrono::steady_clock::now();
  double elapsed_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
  last_update_time_us_ = elapsed_us;
  if (elapsed_us > max_update_time_us_) {
    max_update_time_us_ = elapsed_us;
  }
  update_count_++;

  if (elapsed_us > 900.0) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Update took %.1f us (>900us)! Phase=%s",
                elapsed_us, phaseToString(phase_).c_str());
  }

  if (update_count_ % 100 == 0) {
    std_msgs::msg::Float64 timing_msg;
    timing_msg.data = max_update_time_us_;
    timing_publisher_->publish(timing_msg);

    if (diagnostics_publisher_->get_subscription_count() > 0) {
      sensor_msgs::msg::JointState diag_msg;
      diag_msg.header.stamp = time;
      diag_msg.name.resize(num_joints_ + 1);
      diag_msg.position.resize(num_joints_ + 1);
      diag_msg.velocity.resize(num_joints_ + 1);
      diag_msg.effort.resize(num_joints_ + 1);
      for (int i = 0; i < num_joints_; ++i) {
        diag_msg.name[i] = arm_id_ + "_joint" + std::to_string(i + 1);
        diag_msg.position[i] = q_[i];
        diag_msg.velocity[i] = dq_[i];
        diag_msg.effort[i] = tau_J_d_last_[i];
      }
      diag_msg.name[num_joints_] = "update_time_us";
      diag_msg.position[num_joints_] = last_update_time_us_;
      diag_msg.velocity[num_joints_] = max_update_time_us_;
      diag_msg.effort[num_joints_] = elapsed_us;
      diagnostics_publisher_->publish(diag_msg);
    }

    std_msgs::msg::Float64 force_msg;
    force_msg.data = filtered_force_z_;
    filtered_force_publisher_->publish(force_msg);

    if (command_torques_publisher_->get_subscription_count() > 0) {
      sensor_msgs::msg::JointState cmd_msg;
      cmd_msg.header.stamp = time;
      cmd_msg.name.resize(num_joints_);
      cmd_msg.position.resize(num_joints_);
      cmd_msg.velocity.resize(num_joints_);
      cmd_msg.effort.resize(num_joints_);
      for (int i = 0; i < num_joints_; ++i) {
        cmd_msg.name[i] = arm_id_ + "_joint" + std::to_string(i + 1);
        cmd_msg.position[i] = last_cmd_tau_[i];
        cmd_msg.effort[i] = command_interfaces_[i].get_value();
      }
      command_torques_publisher_->publish(cmd_msg);
    }

    if (force_publisher_->get_subscription_count() > 0) {
      geometry_msgs::msg::WrenchStamped wrench_msg;
      wrench_msg.header.stamp = time;
      wrench_msg.header.frame_id = arm_id_ + "_link0";
      wrench_msg.wrench.force.z = filtered_force_z_;
      if (robot_state_ptr_) {
        wrench_msg.wrench.force.x = robot_state_ptr_->O_F_ext_hat_K[0];
        wrench_msg.wrench.force.y = robot_state_ptr_->O_F_ext_hat_K[1];
      }
      force_publisher_->publish(wrench_msg);
    }
  }

  if (update_count_ % 1000 == 0) {
    double tau_sum = 0.0;
    for (int i = 0; i < num_joints_; ++i) {
      tau_sum += std::abs(last_cmd_tau_[i]);
    }

    if (phase_ == Phase::SETTLE || phase_ == Phase::DESCEND || phase_ == Phase::SLIDE) {
      auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
      Eigen::Vector3d pos = transform.translation();

      RCLCPP_INFO(get_node()->get_logger(),
                  "[%s] cycle=%d tau_sum=%.2f pos=[%.4f,%.4f,%.4f] pos_d=[%.4f,%.4f,%.4f] err_z=%.4f force_z=%.3f K_z=%.0f",
                  phaseToString(phase_).c_str(), update_count_, tau_sum,
                  pos.x(), pos.y(), pos.z(),
                  position_d_.x(), position_d_.y(), position_d_.z(),
                  pos.z() - position_d_.z(), filtered_force_z_,
                  cartesian_stiffness_target_(2, 2));
    } else {
      RCLCPP_INFO(get_node()->get_logger(),
                  "[%s] cycle=%d tau_sum=%.2f t=%.3f",
                  phaseToString(phase_).c_str(), update_count_, tau_sum,
                  joint_elapsed_time_.seconds() / motion_duration_);
    }
  }

  return controller_interface::return_type::OK;
}

/**
 * @brief 更新关节运动阶段
 * 
 * 使用5阶多项式轨迹规划（S形曲线）进行关节空间运动，
 * 将机器人从初始位置移动到目标位置。
 * 当运动完成后，切换到SETTLE阶段。
 * 
 * @param period 控制周期
 * @return 执行成功返回OK
 */
controller_interface::return_type JointForceSlideController::updateJointMove(
    const rclcpp::Duration& period) {
  joint_elapsed_time_ = joint_elapsed_time_ + period;
  double t = joint_elapsed_time_.seconds() / motion_duration_;

  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    double q = state_interfaces_[i].get_value();
    double dq = state_interfaces_[num_joints_ + i].get_value();

    double s;
    if (t >= 1.0) {
      s = 1.0;
    } else {
      s = 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3);
    }

    double q_d = initial_joint_pose_[i] + s * (target_joint_pose_[i] - initial_joint_pose_[i]);
    double dq_d = 0.0;
    if (t < 1.0) {
      dq_d = (30.0 * std::pow(t, 4) - 60.0 * std::pow(t, 3) + 30.0 * std::pow(t, 2)) *
             (target_joint_pose_[i] - initial_joint_pose_[i]) / motion_duration_;
    }

    double tau = k_gains_[i] * (q_d - q) + d_gains_[i] * (dq_d - dq);
    command_interfaces_[i].set_value(tau);
    tau_J_d_last_[i] = tau;
    last_cmd_tau_[i] = tau;
  }

  if (t >= 1.0) {
    phase_ = Phase::SETTLE;
    settle_count_ = 0;
    position_filter_ = filter_params_;

    initImpedanceState();

    RCLCPP_INFO(get_node()->get_logger(),
                "JOINT_MOVE complete -> SETTLE (tau_J_d_last=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f])",
                tau_J_d_last_[0], tau_J_d_last_[1], tau_J_d_last_[2], tau_J_d_last_[3],
                tau_J_d_last_[4], tau_J_d_last_[5], tau_J_d_last_[6]);
  }

  return controller_interface::return_type::OK;
}

/**
 * @brief 更新稳定阶段
 * 
 * 在SETTLE阶段，机器人保持当前位姿，使用高刚度阻抗控制使其稳定。
 * 等待一定周期后记录当前力作为基线力，然后切换到DESCEND阶段。
 * 
 * @return 执行成功返回OK
 */
controller_interface::return_type JointForceSlideController::updateSettle() {
  settle_count_++;

  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  if (settle_count_ == 1) {
    RCLCPP_INFO(get_node()->get_logger(),
                "SETTLE first cycle: tau=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f] err=[%.4f,%.4f,%.4f]",
                tau_d(0), tau_d(1), tau_d(2), tau_d(3), tau_d(4), tau_d(5), tau_d(6),
                error_(0), error_(1), error_(2));
  }

  if (settle_count_ >= settle_cycles_) {
    baseline_force_z_ = filtered_force_z_;
    phase_ = Phase::DESCEND;

    setStiffnessForPhase(Phase::DESCEND);

    auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
    descend_start_z_ = transform.translation().z();

    RCLCPP_INFO(get_node()->get_logger(),
                "SETTLE complete -> DESCEND (baseline_force_z=%.3f, start_z=%.4f)",
                baseline_force_z_, descend_start_z_);
  }

  return controller_interface::return_type::OK;
}

/**
 * @brief 更新下降阶段
 * 
 * 在DESCEND阶段，末端执行器以恒定速度向下移动，直到检测到目标接触力。
 * 使用中等刚度阻抗控制，当接触力达到目标力或下降距离达到最大值时，
 * 切换到SLIDE阶段。
 * 
 * @return 执行成功返回OK
 */
controller_interface::return_type JointForceSlideController::updateDescend() {
  position_d_target_.z() -= descend_speed_ / 1000.0;

  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
  Eigen::Vector3d position(transform.translation());

  double force_change = std::abs(filtered_force_z_ - baseline_force_z_);
  double descended = std::abs(position.z() - descend_start_z_);

  if (force_change >= target_force_ || descended >= max_descend_distance_) {
    phase_ = Phase::SLIDE;
    slide_start_x_ = position.x();
    force_error_integral_ = 0.0;
    z_offset_ = 0.0;

    setStiffnessForPhase(Phase::SLIDE);
    cartesian_stiffness_ = cartesian_stiffness_target_;
    cartesian_damping_ = cartesian_damping_target_;

    double target_offset = target_force_ / translational_stiffness_z_slide_;
    slide_base_z_ = position.z() - target_offset;
    position_d_target_.z() = slide_base_z_;
    position_d_.z() = slide_base_z_;

    position_filter_ = 0.1;

    RCLCPP_INFO(get_node()->get_logger(),
                "DESCEND complete -> SLIDE (force_change=%.3f, descended=%.4f, pos=[%.4f,%.4f,%.4f], slide_base_z=%.4f, target_offset=%.4f)",
                force_change, descended, position.x(), position.y(), position.z(), slide_base_z_, target_offset);
  }

  return controller_interface::return_type::OK;
}

/**
 * @brief 更新滑动阶段
 * 
 * 在SLIDE阶段，末端执行器在保持目标接触力的同时沿X轴滑动。
 * 使用PI控制器调节Z轴位置来维持目标接触力：
 * - 接触力小于目标力时，向下移动增加接触力
 * - 接触力大于目标力时，向上移动减小接触力
 * 
 * 使用低刚度阻抗控制以实现柔顺滑动。
 * 当滑动距离达到目标距离时，切换回JOINT_MOVE阶段。
 * 
 * @return 执行成功返回OK
 */
controller_interface::return_type JointForceSlideController::updateSlide() {
  position_d_target_.x() += slide_speed_ / 1000.0;

  double contact_force = filtered_force_z_ - baseline_force_z_;
  double force_error = target_force_ - contact_force;

  force_error_integral_ += force_error * 0.001;
  force_error_integral_ = std::max(-0.1, std::min(0.1, force_error_integral_));

  z_offset_ = force_kp_ * force_error + force_ki_ * force_error_integral_;
  z_offset_ = std::max(-0.03, std::min(0.03, z_offset_));

  position_d_target_.z() = slide_base_z_ - z_offset_;

  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
  Eigen::Vector3d position(transform.translation());

  double slided = std::abs(position.x() - slide_start_x_);
  double target_slided = std::abs(position_d_target_.x() - slide_start_x_);
  if (slided >= slide_distance_ || target_slided >= slide_distance_ * 1.5) {
    phase_ = Phase::JOINT_MOVE;
    position_filter_ = filter_params_;
    for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
      initial_joint_pose_[i] = q_[i];
    }
    joint_elapsed_time_ = rclcpp::Duration(0, 0);
    force_error_integral_ = 0.0;
    z_offset_ = 0.0;

    RCLCPP_INFO(get_node()->get_logger(),
                "SLIDE complete -> JOINT_MOVE (slided=%.4f, target_slided=%.4f)",
                slided, target_slided);
  }

  return controller_interface::return_type::OK;
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointForceSlideController,
                       controller_interface::ControllerInterface)
