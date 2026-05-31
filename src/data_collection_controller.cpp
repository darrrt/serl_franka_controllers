#include <serl_franka_controllers/data_collection_controller.h>

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

controller_interface::InterfaceConfiguration
DataCollectionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
DataCollectionController::state_interface_configuration() const {
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

CallbackReturn DataCollectionController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");

    auto_declare<double>("grasp_width", 0.0);
    auto_declare<double>("grasp_force", 50.0);
    auto_declare<double>("grasp_speed", 0.1);
    auto_declare<double>("grasp_epsilon_inner", 0.005);
    auto_declare<double>("grasp_epsilon_outer", 0.005);
    auto_declare<int>("max_grasp_attempts", 3);
    auto_declare<std::string>("gripper_action_prefix", "franka_gripper");
    auto_declare<double>("d_min", 0.005);
    auto_declare<double>("d_max", 0.01);
    auto_declare<double>("gripper_open_width", 0.08);

    auto_declare<double>("teach_damping_translational", 10.0);
    auto_declare<double>("teach_damping_rotational", 1.0);

    auto_declare<double>("calib_rise_height", 0.15);
    auto_declare<int>("calib_settle_cycles", 1000);
    auto_declare<std::vector<double>>("calib_home_joint_positions",
        {0.0, 0.168, 0.557, -2.193, -1.1723, 1.186, 0.110});
    auto_declare<std::vector<double>>("calib_home_k_gains",
        {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
    auto_declare<std::vector<double>>("calib_home_d_gains",
        {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0});
    auto_declare<double>("calib_home_motion_duration", 5.0);

    auto_declare<double>("default_dx", 0.01);
    auto_declare<double>("default_dy", 0.0);
    auto_declare<double>("default_dx_step", 0.01);
    auto_declare<double>("default_dy_step", 0.0);
    auto_declare<double>("default_f0", 1.0);

    auto_declare<double>("target_force", 1.0);
    auto_declare<double>("descend_speed", 0.005);
    auto_declare<double>("slide_speed", 0.01);
    auto_declare<double>("force_kp", 0.005);
    auto_declare<double>("force_ki", 0.01);
    auto_declare<double>("force_filter_alpha", 0.1);
    auto_declare<double>("max_descend_distance", 0.25);
    auto_declare<double>("approach_height", 0.05);

    auto_declare<double>("translational_stiffness_xy", 2000.0);
    auto_declare<double>("translational_stiffness_z", 1500.0);
    auto_declare<double>("rotational_stiffness", 150.0);
    auto_declare<double>("translational_damping_xy", 89.0);
    auto_declare<double>("translational_damping_z", 60.0);
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

CallbackReturn DataCollectionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state");

  grasp_width_ = get_node()->get_parameter("grasp_width").as_double();
  grasp_force_ = get_node()->get_parameter("grasp_force").as_double();
  grasp_speed_ = get_node()->get_parameter("grasp_speed").as_double();
  grasp_epsilon_inner_ = get_node()->get_parameter("grasp_epsilon_inner").as_double();
  grasp_epsilon_outer_ = get_node()->get_parameter("grasp_epsilon_outer").as_double();
  max_grasp_attempts_ = get_node()->get_parameter("max_grasp_attempts").as_int();
  gripper_action_prefix_ = get_node()->get_parameter("gripper_action_prefix").as_string();

  d_min_ = get_node()->get_parameter("d_min").as_double();
  d_max_ = get_node()->get_parameter("d_max").as_double();
  gripper_open_width_ = get_node()->get_parameter("gripper_open_width").as_double();

  teach_damping_translational_ = get_node()->get_parameter("teach_damping_translational").as_double();
  teach_damping_rotational_ = get_node()->get_parameter("teach_damping_rotational").as_double();

  calib_rise_height_ = get_node()->get_parameter("calib_rise_height").as_double();
  calib_settle_cycles_ = get_node()->get_parameter("calib_settle_cycles").as_int();

  auto home_jp = get_node()->get_parameter("calib_home_joint_positions").as_double_array();
  if (home_jp.size() == 7) {
    for (size_t i = 0; i < 7; ++i) calib_home_joint_positions_[i] = home_jp[i];
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "calib_home_joint_positions size=%zu, expected 7", home_jp.size());
    calib_home_joint_positions_ = {0.0, 0.168, 0.557, -2.193, -1.1723, 1.186, 0.110};
  }
  auto home_kg = get_node()->get_parameter("calib_home_k_gains").as_double_array();
  if (home_kg.size() == 7) {
    for (size_t i = 0; i < 7; ++i) calib_home_k_gains_[i] = home_kg[i];
  }
  auto home_dg = get_node()->get_parameter("calib_home_d_gains").as_double_array();
  if (home_dg.size() == 7) {
    for (size_t i = 0; i < 7; ++i) calib_home_d_gains_[i] = home_dg[i];
  }
  calib_home_motion_duration_ = get_node()->get_parameter("calib_home_motion_duration").as_double();

  default_dx_ = get_node()->get_parameter("default_dx").as_double();
  default_dy_ = get_node()->get_parameter("default_dy").as_double();
  default_dx_step_ = get_node()->get_parameter("default_dx_step").as_double();
  default_dy_step_ = get_node()->get_parameter("default_dy_step").as_double();
  default_f0_ = get_node()->get_parameter("default_f0").as_double();

  target_force_ = get_node()->get_parameter("target_force").as_double();
  descend_speed_ = get_node()->get_parameter("descend_speed").as_double();
  slide_speed_ = get_node()->get_parameter("slide_speed").as_double();
  force_kp_ = get_node()->get_parameter("force_kp").as_double();
  force_ki_ = get_node()->get_parameter("force_ki").as_double();
  force_filter_alpha_ = get_node()->get_parameter("force_filter_alpha").as_double();
  max_descend_distance_ = get_node()->get_parameter("max_descend_distance").as_double();
  approach_height_ = get_node()->get_parameter("approach_height").as_double();

  translational_stiffness_xy_ = get_node()->get_parameter("translational_stiffness_xy").as_double();
  translational_stiffness_z_ = get_node()->get_parameter("translational_stiffness_z").as_double();
  rotational_stiffness_ = get_node()->get_parameter("rotational_stiffness").as_double();
  translational_damping_xy_ = get_node()->get_parameter("translational_damping_xy").as_double();
  translational_damping_z_ = get_node()->get_parameter("translational_damping_z").as_double();
  rotational_damping_ = get_node()->get_parameter("rotational_damping").as_double();
  nullspace_stiffness_ = get_node()->get_parameter("nullspace_stiffness").as_double();
  nullspace_stiffness_target_ = nullspace_stiffness_;
  joint1_nullspace_stiffness_ = get_node()->get_parameter("joint1_nullspace_stiffness").as_double();
  joint1_nullspace_stiffness_target_ = joint1_nullspace_stiffness_;

  max_joint_velocity_ = get_node()->get_parameter("max_joint_velocity").as_double();
  max_cartesian_velocity_ = get_node()->get_parameter("max_cartesian_velocity").as_double();
  max_torque_ = get_node()->get_parameter("max_torque").as_double();

  calib_orientations_.clear();
  calib_orientations_.push_back(Eigen::Quaterniond::Identity());
  calib_orientations_.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitX())));
  calib_orientations_.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitX())));
  calib_orientations_.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 6, Eigen::Vector3d::UnitY())));
  calib_orientations_.push_back(Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI / 6, Eigen::Vector3d::UnitY())));

  RCLCPP_INFO(get_node()->get_logger(),
              "DataCollectionController configured: grasp_force=%.1fN, calib_rise=%.2fm, "
              "default_f0=%.1fN, calib_configs=%zu, d_min=%.4f, d_max=%.4f, gripper_open_width=%.4f",
              grasp_force_, calib_rise_height_, default_f0_, calib_orientations_.size(),
              d_min_, d_max_, gripper_open_width_);

  phase_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
      "~/phase", rclcpp::SystemDefaultsQoS());
  completion_publisher_ = get_node()->create_publisher<std_msgs::msg::Bool>(
      "~/completion", rclcpp::SystemDefaultsQoS());
  filtered_force_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
      "~/filtered_force_z", rclcpp::SystemDefaultsQoS());
  diagnostics_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      "~/diagnostics", rclcpp::SystemDefaultsQoS());
  force_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/force_z", rclcpp::SystemDefaultsQoS());
  payload_mass_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64>(
      "~/payload_mass", rclcpp::SystemDefaultsQoS());
  payload_com_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/payload_com", rclcpp::SystemDefaultsQoS());

  grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
      get_node(), gripper_action_prefix_ + "/grasp");
  move_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
      get_node(), gripper_action_prefix_ + "/move");

  gripper_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() { gripperTimerCallback(); });

  teach_trigger_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/teach_trigger", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { teachTriggerCallback(msg); });

  collection_params_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/collection_params", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) { collectionParamsCallback(msg); });

  reset_subscriber_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/reset", rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { resetCallback(msg); });

  gripper_state_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      gripper_action_prefix_ + "/joint_states", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { gripperStateCallback(msg); });

  return CallbackReturn::SUCCESS;
}

CallbackReturn DataCollectionController::on_activate(
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

  phase_ = Phase::GRASP;
  grasp_attempt_count_ = 0;
  grasp_state_ = GraspState::IDLE;
  grasp_request_ = false;
  open_request_ = false;
  grasp_succeeded_ = false;
  grasp_sub_phase_ = GraspSubPhase::OPEN;
  move_state_ = MoveState::IDLE;
  move_request_ = false;
  grasp_target_width_ = d_min_;
  move_target_width_ = gripper_open_width_;
  gripper_width_ = gripper_open_width_;
  gripper_width_valid_ = false;
  teach_triggered_ = false;
  filtered_force_z_ = 0.0;
  force_error_integral_ = 0.0;
  z_offset_ = 0.0;
  safety_violation_ = false;
  update_count_ = 0;
  tau_J_d_last_.fill(0.0);
  last_cmd_tau_.fill(0.0);

  collection_dx_ = default_dx_;
  collection_dy_ = default_dy_;
  collection_dx_step_ = default_dx_step_;
  collection_dy_step_ = default_dy_step_;
  collection_f0_ = default_f0_;
  collection_params_received_ = false;

  initImpedanceState();

  RCLCPP_INFO(get_node()->get_logger(),
              "DataCollectionController activated: phase=GRASP, robot_state_ptr=%p",
              static_cast<void*>(robot_state_ptr_));

  if (gripper_timer_) {
    gripper_timer_->reset();
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn DataCollectionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (gripper_timer_) {
    gripper_timer_->cancel();
  }
  franka_robot_model_->release_interfaces();
  robot_state_ptr_ = nullptr;
  robot_state_interface_index_ = -1;
  return CallbackReturn::SUCCESS;
}

void DataCollectionController::updateJointStates() {
  for (int i = 0; i < num_joints_; ++i) {
    q_.at(i) = state_interfaces_[i].get_value();
    dq_.at(i) = state_interfaces_[num_joints_ + i].get_value();
  }
}

franka::RobotState* DataCollectionController::getRobotStatePtr() {
  if (robot_state_interface_index_ < 0) {
    return nullptr;
  }
  double val = state_interfaces_[robot_state_interface_index_].get_value();
  franka::RobotState* ptr = nullptr;
  std::memcpy(&ptr, &val, sizeof(ptr));
  return ptr;
}

Eigen::Matrix<double, 7, 1> DataCollectionController::saturateTorqueRate(
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

std::string DataCollectionController::phaseToString(Phase p) {
  switch (p) {
    case Phase::GRASP: return "GRASP";
    case Phase::TEACH: return "TEACH";
    case Phase::CALIBRATE: return "CALIBRATE";
    case Phase::APPROACH: return "APPROACH";
    case Phase::DESCEND: return "DESCEND";
    case Phase::SLIDE: return "SLIDE";
    case Phase::LIFT: return "LIFT";
    case Phase::WAIT_PARAMS: return "WAIT_PARAMS";
    case Phase::DONE: return "DONE";
    default: return "UNKNOWN";
  }
}

void DataCollectionController::setStiffnessForPhase(Phase phase) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_damping_target_.setIdentity();

  switch (phase) {
    case Phase::TEACH:
      cartesian_stiffness_target_.topLeftCorner(3, 3) = Eigen::Matrix3d::Zero();
      cartesian_stiffness_target_.bottomRightCorner(3, 3) = Eigen::Matrix3d::Zero();
      cartesian_damping_target_.topLeftCorner(3, 3) =
          teach_damping_translational_ * Eigen::Matrix3d::Identity();
      cartesian_damping_target_.bottomRightCorner(3, 3) =
          teach_damping_rotational_ * Eigen::Matrix3d::Identity();
      break;
    case Phase::DESCEND:
      cartesian_stiffness_target_.topLeftCorner(3, 3) =
          translational_stiffness_xy_ * Eigen::Matrix3d::Identity();
      cartesian_stiffness_target_(2, 2) = translational_stiffness_z_ * 0.5;
      cartesian_stiffness_target_.bottomRightCorner(3, 3) =
          rotational_stiffness_ * Eigen::Matrix3d::Identity();
      cartesian_damping_target_.topLeftCorner(3, 3) =
          translational_damping_xy_ * Eigen::Matrix3d::Identity();
      cartesian_damping_target_(2, 2) = translational_damping_z_ * 0.5;
      cartesian_damping_target_.bottomRightCorner(3, 3) =
          rotational_damping_ * Eigen::Matrix3d::Identity();
      break;
    case Phase::SLIDE:
      cartesian_stiffness_target_.topLeftCorner(3, 3) =
          translational_stiffness_xy_ * Eigen::Matrix3d::Identity();
      cartesian_stiffness_target_(2, 2) = translational_stiffness_z_ * 0.3;
      cartesian_stiffness_target_.bottomRightCorner(3, 3) =
          rotational_stiffness_ * Eigen::Matrix3d::Identity();
      cartesian_damping_target_.topLeftCorner(3, 3) =
          translational_damping_xy_ * Eigen::Matrix3d::Identity();
      cartesian_damping_target_(2, 2) = translational_damping_z_ * 0.3;
      cartesian_damping_target_.bottomRightCorner(3, 3) =
          rotational_damping_ * Eigen::Matrix3d::Identity();
      break;
    default:
      cartesian_stiffness_target_.topLeftCorner(3, 3) =
          translational_stiffness_xy_ * Eigen::Matrix3d::Identity();
      cartesian_stiffness_target_(2, 2) = translational_stiffness_z_;
      cartesian_stiffness_target_.bottomRightCorner(3, 3) =
          rotational_stiffness_ * Eigen::Matrix3d::Identity();
      cartesian_damping_target_.topLeftCorner(3, 3) =
          translational_damping_xy_ * Eigen::Matrix3d::Identity();
      cartesian_damping_target_(2, 2) = translational_damping_z_;
      cartesian_damping_target_.bottomRightCorner(3, 3) =
          rotational_damping_ * Eigen::Matrix3d::Identity();
      break;
  }
}

void DataCollectionController::initImpedanceState() {
  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
  position_d_ = transform.translation();
  orientation_d_ = Eigen::Quaterniond(transform.linear());
  position_d_target_ = position_d_;
  orientation_d_target_ = orientation_d_;

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_current(q_.data());
  q_d_nullspace_ = q_current;
  error_i_.setZero();

  setStiffnessForPhase(phase_);

  cartesian_stiffness_ = cartesian_stiffness_target_;
  cartesian_damping_ = cartesian_damping_target_;
}

void DataCollectionController::computeImpedanceControl(
    Eigen::Matrix<double, 7, 1>& tau_d) {
  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_.data());

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

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d_last(tau_J_d_last_.data());
  tau_d = saturateTorqueRate(tau_d, tau_J_d_last);

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

void DataCollectionController::computeGravityCompensation(
    Eigen::Matrix<double, 7, 1>& tau_d) {
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_.data());

  Eigen::Matrix<double, 6, 1> damping_force =
      -teach_damping_translational_ * jacobian.topRows(3) * dq;
  Eigen::Matrix<double, 6, 1> damping_torque =
      -teach_damping_rotational_ * jacobian.bottomRows(3) * dq;
  Eigen::Matrix<double, 6, 1> cartesian_damping_wrench;
  cartesian_damping_wrench << damping_force, damping_torque;

  tau_d = coriolis + jacobian.transpose() * cartesian_damping_wrench;

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d_last(tau_J_d_last_.data());
  tau_d = saturateTorqueRate(tau_d, tau_J_d_last);

  for (size_t i = 0; i < 7; ++i) {
    tau_J_d_last_[i] = tau_d(i);
  }
}

bool DataCollectionController::checkSafetyLimits() {
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

void DataCollectionController::gripperTimerCallback() {
  if (grasp_request_.exchange(false)) {
    sendGripperGrasp();
  }
  if (move_request_.exchange(false)) {
    sendGripperOpen();
  }
}

void DataCollectionController::sendGripperGrasp() {
  if (!grasp_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_node()->get_logger(), "Grasp action server not available at %s/grasp",
                gripper_action_prefix_.c_str());
    grasp_state_ = GraspState::FAILED;
    return;
  }

  auto goal_msg = franka_msgs::action::Grasp::Goal();
  goal_msg.width = grasp_target_width_;
  goal_msg.speed = grasp_speed_;
  goal_msg.force = grasp_force_;
  goal_msg.epsilon.inner = grasp_epsilon_inner_;
  goal_msg.epsilon.outer = grasp_epsilon_outer_;

  auto send_goal_options = rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::SharedPtr&
                 goal_handle) {
        this->graspGoalResponseCallback(goal_handle);
      };
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                 result) { this->graspResultCallback(result); };

  grasp_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(get_node()->get_logger(),
              "Sending grasp command: width=%.4f, force=%.1f, speed=%.3f",
              grasp_target_width_, grasp_force_, grasp_speed_);
}

void DataCollectionController::sendGripperOpen() {
  if (!move_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(get_node()->get_logger(), "Move action server not available at %s/move",
                gripper_action_prefix_.c_str());
    return;
  }

  auto goal_msg = franka_msgs::action::Move::Goal();
  goal_msg.width = move_target_width_;
  goal_msg.speed = grasp_speed_;

  auto send_goal_options = rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::SharedPtr&
                 goal_handle) { this->moveGoalResponseCallback(goal_handle); };
  send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult&
                 result) { this->moveResultCallback(result); };

  move_client_->async_send_goal(goal_msg, send_goal_options);
  RCLCPP_INFO(get_node()->get_logger(), "Sending gripper open command: width=%.4f", move_target_width_);
}

void DataCollectionController::graspGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(get_node()->get_logger(), "Grasp goal rejected");
    grasp_state_ = GraspState::FAILED;
  } else {
    grasp_state_ = GraspState::WAITING;
    RCLCPP_INFO(get_node()->get_logger(), "Grasp goal accepted");
  }
}

void DataCollectionController::graspResultCallback(
    const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult& result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Grasp succeeded (object detected)");
    grasp_succeeded_ = true;
    grasp_state_ = GraspState::SUCCEEDED;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Grasp failed (no object or error: %s)",
                result.result->error.c_str());
    grasp_succeeded_ = false;
    grasp_state_ = GraspState::FAILED;
  }
}

void DataCollectionController::moveGoalResponseCallback(
    const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(get_node()->get_logger(), "Move goal rejected");
    move_state_ = MoveState::FAILED;
  } else {
    move_state_ = MoveState::WAITING;
    RCLCPP_INFO(get_node()->get_logger(), "Move goal accepted");
  }
}

void DataCollectionController::moveResultCallback(
    const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Gripper move succeeded");
    move_state_ = MoveState::SUCCEEDED;
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Gripper move failed: %s",
                result.result->error.c_str());
    move_state_ = MoveState::FAILED;
  }
}

void DataCollectionController::gripperStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->position.size() >= 2) {
    gripper_width_ = std::abs(msg->position[0]) + std::abs(msg->position[1]);
    gripper_width_valid_ = true;
  }
}

void DataCollectionController::buildGravityRegressor(
    const Eigen::Matrix3d& R_EE,
    Eigen::Matrix<double, 6, 4>& Y) {
  Eigen::Vector3d gravity_dir_ee = R_EE.transpose() * Eigen::Vector3d(0.0, 0.0, -1.0);
  double r1 = gravity_dir_ee.x();
  double r2 = gravity_dir_ee.y();
  double r3 = gravity_dir_ee.z();
  double g = 9.81;

  Y.setZero();
  Y(0, 0) = g * r1;
  Y(1, 0) = g * r2;
  Y(2, 0) = g * r3;
  Y(3, 2) = g * r3;
  Y(3, 3) = -g * r2;
  Y(4, 1) = -g * r3;
  Y(4, 3) = g * r1;
  Y(5, 1) = g * r2;
  Y(5, 2) = -g * r1;
}

void DataCollectionController::solvePayloadCalibration() {
  int n = calib_record_count_;
  if (n < 4) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Not enough calibration points (%d < 4), using defaults", n);
    payload_mass_ = 0.0;
    payload_com_ = Eigen::Vector3d::Zero();
    return;
  }

  Eigen::MatrixXd Y_full(6 * n, 4);
  Eigen::VectorXd W_full(6 * n);
  Y_full = calib_Y_stack_.topRows(6 * n);
  W_full = calib_W_stack_.head(6 * n);

  Eigen::Matrix<double, 4, 4> YtY = Y_full.transpose() * Y_full;
  Eigen::Matrix<double, 4, 1> pi = YtY.ldlt().solve(Y_full.transpose() * W_full);

  payload_mass_ = pi(0);
  if (std::abs(payload_mass_) > 1e-6) {
    payload_com_(0) = pi(1) / payload_mass_;
    payload_com_(1) = pi(2) / payload_mass_;
    payload_com_(2) = pi(3) / payload_mass_;
  } else {
    payload_com_.setZero();
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "Payload calibration result: mass=%.4f kg, CoM=[%.4f, %.4f, %.4f]",
              payload_mass_, payload_com_.x(), payload_com_.y(), payload_com_.z());

  std_msgs::msg::Float64 mass_msg;
  mass_msg.data = payload_mass_;
  payload_mass_publisher_->publish(mass_msg);

  std_msgs::msg::Float64MultiArray com_msg;
  com_msg.data = {payload_com_.x(), payload_com_.y(), payload_com_.z()};
  payload_com_publisher_->publish(com_msg);
}

void DataCollectionController::teachTriggerCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && phase_ == Phase::TEACH) {
    teach_triggered_ = true;
    RCLCPP_INFO(get_node()->get_logger(), "TEACH trigger received");
  }
}

void DataCollectionController::collectionParamsCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() >= 5) {
    collection_dx_ = msg->data[0];
    collection_dy_ = msg->data[1];
    collection_dx_step_ = msg->data[2];
    collection_dy_step_ = msg->data[3];
    collection_f0_ = msg->data[4];
    collection_params_received_ = true;
    RCLCPP_INFO(get_node()->get_logger(),
                "Collection params received: dx=%.4f dy=%.4f dx_step=%.4f dy_step=%.4f F0=%.2f",
                collection_dx_, collection_dy_, collection_dx_step_,
                collection_dy_step_, collection_f0_);
  } else {
    RCLCPP_WARN(get_node()->get_logger(),
                "Collection params need 5 values, got %zu", msg->data.size());
  }
}

void DataCollectionController::resetCallback(
    const std_msgs::msg::Bool::SharedPtr /*msg*/) {
  phase_ = Phase::GRASP;
  grasp_attempt_count_ = 0;
  grasp_state_ = GraspState::IDLE;
  grasp_succeeded_ = false;
  grasp_sub_phase_ = GraspSubPhase::OPEN;
  move_state_ = MoveState::IDLE;
  grasp_target_width_ = d_min_;
  move_target_width_ = gripper_open_width_;
  gripper_width_ = gripper_open_width_;
  gripper_width_valid_ = false;
  teach_triggered_ = false;
  filtered_force_z_ = 0.0;
  force_error_integral_ = 0.0;
  z_offset_ = 0.0;
  safety_violation_ = false;
  collection_params_received_ = false;

  initImpedanceState();
  RCLCPP_INFO(get_node()->get_logger(), "Controller reset -> GRASP");
}

controller_interface::return_type DataCollectionController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
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
    case Phase::GRASP: updateGrasp(); break;
    case Phase::TEACH: updateTeach(); break;
    case Phase::CALIBRATE: updateCalibrate(); break;
    case Phase::APPROACH: updateApproach(); break;
    case Phase::DESCEND: updateDescend(); break;
    case Phase::SLIDE: updateSlide(); break;
    case Phase::LIFT: updateLift(); break;
    case Phase::WAIT_PARAMS: updateWaitParams(); break;
    case Phase::DONE: updateDone(); break;
  }

  if (checkSafetyLimits()) {
    safety_violation_ = true;
    RCLCPP_ERROR(get_node()->get_logger(), "Safety limit violated! Controller stopped.");
  }

  update_count_++;

  if (update_count_ % 100 == 0) {
    std_msgs::msg::String phase_msg;
    phase_msg.data = phaseToString(phase_);
    phase_publisher_->publish(phase_msg);

    std_msgs::msg::Float64 force_msg;
    force_msg.data = filtered_force_z_;
    filtered_force_publisher_->publish(force_msg);

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
    auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
    Eigen::Vector3d pos = transform.translation();

    RCLCPP_INFO(get_node()->get_logger(),
                "[%s] cycle=%d pos=[%.4f, %.4f, %.4f] pos_d=[%.4f, %.4f, %.4f] force_z=%.3f",
                phaseToString(phase_).c_str(), update_count_,
                pos.x(), pos.y(), pos.z(),
                position_d_target_.x(), position_d_target_.y(), position_d_target_.z(),
                filtered_force_z_);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateGrasp() {
  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  switch (grasp_sub_phase_) {
    case GraspSubPhase::OPEN: {
      if (move_state_ == MoveState::IDLE) {
        grasp_attempt_count_++;
        if (grasp_attempt_count_ > max_grasp_attempts_) {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Max grasp attempts (%d) reached, proceeding without object",
                      max_grasp_attempts_);
          phase_ = Phase::TEACH;
          setStiffnessForPhase(Phase::TEACH);
          initImpedanceState();
          RCLCPP_INFO(get_node()->get_logger(), "GRASP -> TEACH (max attempts reached)");
          break;
        }
        move_target_width_ = gripper_open_width_;
        move_request_ = true;
        move_state_ = MoveState::SENDING;
        RCLCPP_INFO(get_node()->get_logger(),
                    "GRASP::OPEN: attempt %d/%d, opening gripper to %.4f",
                    grasp_attempt_count_, max_grasp_attempts_, gripper_open_width_);
      }
      if (move_state_ == MoveState::SUCCEEDED || move_state_ == MoveState::FAILED) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "GRASP::OPEN -> CLOSING (move %s)",
                    move_state_ == MoveState::SUCCEEDED ? "succeeded" : "failed");
        move_state_ = MoveState::IDLE;
        grasp_sub_phase_ = GraspSubPhase::CLOSING;
        grasp_state_ = GraspState::IDLE;
      }
      break;
    }

    case GraspSubPhase::CLOSING: {
      if (grasp_state_ == GraspState::IDLE) {
        grasp_target_width_ = d_min_;
        grasp_request_ = true;
        grasp_state_ = GraspState::SENDING;
        RCLCPP_INFO(get_node()->get_logger(),
                    "GRASP::CLOSING: closing gripper to d_min=%.4f, force=%.1f",
                    d_min_, grasp_force_);
      }
      if (grasp_state_ == GraspState::SUCCEEDED || grasp_state_ == GraspState::FAILED) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "GRASP::CLOSING -> CHECK (grasp %s, gripper_width=%.4f)",
                    grasp_state_ == GraspState::SUCCEEDED ? "succeeded" : "failed",
                    gripper_width_);
        grasp_sub_phase_ = GraspSubPhase::CHECK;
      }
      break;
    }

    case GraspSubPhase::CHECK: {
      if (gripper_width_valid_ && gripper_width_ >= d_min_ && gripper_width_ <= d_max_) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "GRASP::CHECK: object detected (width=%.4f in [%.4f, %.4f])",
                    gripper_width_, d_min_, d_max_);
        grasp_succeeded_ = true;
        phase_ = Phase::TEACH;
        setStiffnessForPhase(Phase::TEACH);
        initImpedanceState();
        RCLCPP_INFO(get_node()->get_logger(), "GRASP -> TEACH (object grasped)");
      } else if (!gripper_width_valid_ &&
                 grasp_state_ == GraspState::SUCCEEDED) {
        RCLCPP_INFO(get_node()->get_logger(),
                    "GRASP::CHECK: gripper width not available, grasp action succeeded, assuming object");
        grasp_succeeded_ = true;
        phase_ = Phase::TEACH;
        setStiffnessForPhase(Phase::TEACH);
        initImpedanceState();
        RCLCPP_INFO(get_node()->get_logger(), "GRASP -> TEACH (grasp succeeded, no width data)");
      } else {
        RCLCPP_INFO(get_node()->get_logger(),
                    "GRASP::CHECK: no object (width=%.4f not in [%.4f, %.4f]), attempt %d/%d",
                    gripper_width_, d_min_, d_max_,
                    grasp_attempt_count_, max_grasp_attempts_);
        grasp_sub_phase_ = GraspSubPhase::OPEN;
        move_state_ = MoveState::IDLE;
        grasp_state_ = GraspState::IDLE;
      }
      break;
    }
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateTeach() {
  Eigen::Matrix<double, 7, 1> tau_d;
  computeGravityCompensation(tau_d);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  if (teach_triggered_.load()) {
    teach_triggered_.store(false);

    auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
    teach_start_position_ = transform.translation();
    teach_start_orientation_ = Eigen::Quaterniond(transform.linear());
    slide_base_z_ = teach_start_position_.z();

    calib_rise_position_ = teach_start_position_;
    calib_rise_position_.z() += calib_rise_height_;
    calib_rise_orientation_ = teach_start_orientation_;

    RCLCPP_INFO(get_node()->get_logger(),
                "TEACH trigger: start_pos=[%.4f, %.4f, %.4f], calib_rise_target=[%.4f, %.4f, %.4f]",
                teach_start_position_.x(), teach_start_position_.y(),
                teach_start_position_.z(),
                calib_rise_position_.x(), calib_rise_position_.y(),
                calib_rise_position_.z());

    phase_ = Phase::CALIBRATE;
    calib_sub_phase_ = CalibSubPhase::RISE;
    calib_current_config_ = 0;
    calib_record_count_ = 0;
    calib_settle_count_ = 0;
    calib_home_cycle_count_ = 0;

    int n = static_cast<int>(calib_orientations_.size());
    calib_Y_stack_.resize(6 * n, 4);
    calib_W_stack_.resize(6 * n);
    calib_Y_stack_.setZero();
    calib_W_stack_.setZero();

    setStiffnessForPhase(Phase::CALIBRATE);
    initImpedanceState();

    RCLCPP_INFO(get_node()->get_logger(), "TEACH -> CALIBRATE");
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateCalibrate() {
  Eigen::Matrix<double, 7, 1> tau_d;

  switch (calib_sub_phase_) {
    case CalibSubPhase::RISE: {
      auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
      Eigen::Vector3d current_pos = transform.translation();

      position_d_target_ = calib_rise_position_;
      orientation_d_target_ = calib_rise_orientation_;

      computeImpedanceControl(tau_d);
      for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d(i));
        last_cmd_tau_[i] = tau_d(i);
      }

      if ((current_pos - calib_rise_position_).norm() < 0.015) {
        calib_sub_phase_ = CalibSubPhase::MOVE_TO_HOME_CONFIG;
        for (size_t i = 0; i < 7; ++i) {
          calib_home_initial_joint_pose_[i] = q_[i];
        }
        calib_home_cycle_count_ = 0;
        RCLCPP_INFO(get_node()->get_logger(),
                    "CALIBRATE: risen to safe height z=%.4f, moving to home config",
                    calib_rise_position_.z());
      }
      break;
    }

    case CalibSubPhase::MOVE_TO_HOME_CONFIG: {
      calib_home_cycle_count_++;
      double t = static_cast<double>(calib_home_cycle_count_) /
                 (calib_home_motion_duration_ * 1000.0);

      for (size_t i = 0; i < 7; ++i) {
        double q_cur = state_interfaces_[i].get_value();
        double dq_cur = state_interfaces_[num_joints_ + i].get_value();

        double s;
        if (t >= 1.0) {
          s = 1.0;
        } else {
          s = 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3);
        }

        double q_d = calib_home_initial_joint_pose_[i] +
                     s * (calib_home_joint_positions_[i] - calib_home_initial_joint_pose_[i]);
        double dq_d = 0.0;
        if (t < 1.0) {
          dq_d = (30.0 * std::pow(t, 4) - 60.0 * std::pow(t, 3) + 30.0 * std::pow(t, 2)) *
                 (calib_home_joint_positions_[i] - calib_home_initial_joint_pose_[i]) /
                 calib_home_motion_duration_;
        }

        double tau = calib_home_k_gains_[i] * (q_d - q_cur) +
                     calib_home_d_gains_[i] * (dq_d - dq_cur);
        command_interfaces_[i].set_value(tau);
        last_cmd_tau_[i] = tau;
        tau_J_d_last_[i] = tau;
      }

      if (t >= 1.0) {
        bool close_enough = true;
        for (size_t i = 0; i < 7; ++i) {
          double q_cur = state_interfaces_[i].get_value();
          if (std::abs(q_cur - calib_home_joint_positions_[i]) > 0.05) {
            close_enough = false;
            break;
          }
        }

        if (close_enough) {
          auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
          Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
          calib_rise_position_ = transform.translation();
          calib_rise_orientation_ = Eigen::Quaterniond(transform.linear());

          initImpedanceState();

          calib_sub_phase_ = CalibSubPhase::MOVE_TO_CONFIG;
          calib_current_config_ = 0;
          calib_settle_count_ = 0;

          RCLCPP_INFO(get_node()->get_logger(),
                      "CALIBRATE: reached home config, calib_base_pos=[%.4f, %.4f, %.4f]",
                      calib_rise_position_.x(), calib_rise_position_.y(),
                      calib_rise_position_.z());
        }
      }
      break;
    }

    case CalibSubPhase::MOVE_TO_CONFIG: {
      if (calib_current_config_ >= static_cast<int>(calib_orientations_.size())) {
        calib_sub_phase_ = CalibSubPhase::SOLVE;
        RCLCPP_INFO(get_node()->get_logger(), "CALIBRATE: all configs recorded, solving...");
        break;
      }

      Eigen::Quaterniond target_orient = calib_rise_orientation_ * calib_orientations_[calib_current_config_];
      position_d_target_ = calib_rise_position_;
      orientation_d_target_ = target_orient;

      computeImpedanceControl(tau_d);
      for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d(i));
        last_cmd_tau_[i] = tau_d(i);
      }

      auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
      Eigen::Quaterniond current_orient(transform.linear());

      double orient_error = current_orient.angularDistance(target_orient); // 1. 姿态比较接近了（放宽到0.15 rad）
      // bool robot_stopped = dq_.norm() < 0.01; // 或者末端速度很小
      // 引入 Eigen 映射来计算 std::array 的范数
      bool robot_stopped = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(dq_.data()).norm() < 0.01;
      if (orient_error < 0.15 && robot_stopped) {
        calib_sub_phase_ = CalibSubPhase::SETTLE;
        calib_settle_count_ = 0;
        RCLCPP_INFO(get_node()->get_logger(),
                    "CALIBRATE: reached config %d/%zu, settling...",
                    calib_current_config_ + 1, calib_orientations_.size());
      }
      break;
    }

    case CalibSubPhase::SETTLE: {
      position_d_target_ = calib_rise_position_;
      orientation_d_target_ = calib_rise_orientation_ * calib_orientations_[calib_current_config_];

      computeImpedanceControl(tau_d);
      for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d(i));
        last_cmd_tau_[i] = tau_d(i);
      }

      calib_settle_count_++;
      if (calib_settle_count_ >= calib_settle_cycles_) {
        calib_sub_phase_ = CalibSubPhase::RECORD;
        RCLCPP_INFO(get_node()->get_logger(),
                    "CALIBRATE: settled at config %d, recording...", calib_current_config_ + 1);
      }
      break;
    }

    case CalibSubPhase::RECORD: {
      auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
      Eigen::Matrix3d R_EE = transform.linear();

      Eigen::Matrix<double, 6, 4> Y_i;
      buildGravityRegressor(R_EE, Y_i);

      Eigen::Matrix<double, 6, 1> W_ee = Eigen::Matrix<double, 6, 1>::Zero();
      if (robot_state_ptr_) {
        Eigen::Vector3d F_base(robot_state_ptr_->O_F_ext_hat_K[0],
                               robot_state_ptr_->O_F_ext_hat_K[1],
                               robot_state_ptr_->O_F_ext_hat_K[2]);
        Eigen::Vector3d Tau_base(robot_state_ptr_->O_F_ext_hat_K[3],
                                  robot_state_ptr_->O_F_ext_hat_K[4],
                                  robot_state_ptr_->O_F_ext_hat_K[5]);
        W_ee.head(3) = R_EE.transpose() * F_base;
        W_ee.tail(3) = R_EE.transpose() * Tau_base;
      }

      int idx = calib_record_count_;
      calib_Y_stack_.block(6 * idx, 0, 6, 4) = Y_i;
      calib_W_stack_.segment(6 * idx, 6) = W_ee;
      calib_record_count_++;

      RCLCPP_INFO(get_node()->get_logger(),
                  "CALIBRATE: recorded config %d, W_ee=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                  calib_current_config_ + 1,
                  W_ee(0), W_ee(1), W_ee(2), W_ee(3), W_ee(4), W_ee(5));

      calib_current_config_++;
      calib_sub_phase_ = CalibSubPhase::MOVE_TO_CONFIG;
      calib_settle_count_ = 0;
      break;
    }

    case CalibSubPhase::SOLVE: {
      solvePayloadCalibration();

      position_d_target_ = teach_start_position_;
      orientation_d_target_ = teach_start_orientation_;

      computeImpedanceControl(tau_d);
      for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d(i));
        last_cmd_tau_[i] = tau_d(i);
      }

      auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
      Eigen::Vector3d current_pos = transform.translation();

      if ((current_pos - teach_start_position_).norm() < 0.01) {
        calib_sub_phase_ = CalibSubPhase::FINISHED;
        phase_ = Phase::WAIT_PARAMS;
        collection_params_received_ = false;
        setStiffnessForPhase(Phase::APPROACH);
        initImpedanceState();

        RCLCPP_INFO(get_node()->get_logger(),
                    "CALIBRATE -> WAIT_PARAMS (returned to start pos)");
      }
      break;
    }

    case CalibSubPhase::RETURN: {
      position_d_target_ = teach_start_position_;
      orientation_d_target_ = teach_start_orientation_;

      computeImpedanceControl(tau_d);
      for (size_t i = 0; i < 7; ++i) {
        command_interfaces_[i].set_value(tau_d(i));
        last_cmd_tau_[i] = tau_d(i);
      }

      auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
      Eigen::Vector3d current_pos = transform.translation();

      if ((current_pos - teach_start_position_).norm() < 0.01) {
        calib_sub_phase_ = CalibSubPhase::FINISHED;
        phase_ = Phase::WAIT_PARAMS;
        collection_params_received_ = false;
        setStiffnessForPhase(Phase::APPROACH);
        initImpedanceState();
        RCLCPP_INFO(get_node()->get_logger(), "CALIBRATE -> WAIT_PARAMS");
      }
      break;
    }

    case CalibSubPhase::FINISHED:
      break;
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateApproach() {
  Eigen::Vector3d target_pos(
      teach_start_position_.x() + collection_dx_,
      teach_start_position_.y() + collection_dy_,
      slide_base_z_ + approach_height_);

  position_d_target_ = target_pos;
  orientation_d_target_ = teach_start_orientation_;

  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
  Eigen::Vector3d current_pos = transform.translation();

  if ((current_pos - target_pos).norm() < 0.003) {
    phase_ = Phase::DESCEND;
    baseline_force_z_ = filtered_force_z_;
    force_error_integral_ = 0.0;
    z_offset_ = 0.0;

    auto pose_matrix2 = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Affine3d transform2(Eigen::Matrix4d::Map(pose_matrix2.data()));
    descend_start_z_ = transform2.translation().z();

    setStiffnessForPhase(Phase::DESCEND);
    RCLCPP_INFO(get_node()->get_logger(),
                "APPROACH -> DESCEND (target=[%.4f, %.4f, %.4f], baseline_force=%.3f)",
                target_pos.x(), target_pos.y(), target_pos.z(), baseline_force_z_);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateDescend() {
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

  if (force_change >= collection_f0_ || descended >= max_descend_distance_) {
    phase_ = Phase::SLIDE;
    slide_base_z_ = position.z();
    position_d_target_.z() = slide_base_z_;
    position_d_.z() = slide_base_z_;
    force_error_integral_ = 0.0;
    z_offset_ = 0.0;

    slide_start_xy_ = Eigen::Vector3d(position.x(), position.y(), slide_base_z_);
    slide_target_xy_ = Eigen::Vector3d(
        teach_start_position_.x() + collection_dx_ + collection_dx_step_,
        teach_start_position_.y() + collection_dy_ + collection_dy_step_,
        slide_base_z_);

    setStiffnessForPhase(Phase::SLIDE);
    cartesian_stiffness_ = cartesian_stiffness_target_;
    cartesian_damping_ = cartesian_damping_target_;
    position_filter_ = 0.1;

    RCLCPP_INFO(get_node()->get_logger(),
                "DESCEND -> SLIDE (force_change=%.3f, descended=%.4f, pos=[%.4f, %.4f, %.4f])",
                force_change, descended, position.x(), position.y(), position.z());
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateSlide() {
  Eigen::Vector2d target_xy(slide_target_xy_.x(), slide_target_xy_.y());
  Eigen::Vector2d current_xy(position_d_target_.x(), position_d_target_.y());
  Eigen::Vector2d direction = target_xy - current_xy;
  double distance = direction.norm();

  if (distance < 0.001) {
    phase_ = Phase::LIFT;
    setStiffnessForPhase(Phase::LIFT);
    RCLCPP_INFO(get_node()->get_logger(),
                "SLIDE -> LIFT (slide complete, target reached)");
    return controller_interface::return_type::OK;
  }

  direction.normalize();
  double step = std::min(slide_speed_ / 1000.0, distance);

  position_d_target_.x() += direction.x() * step;
  position_d_target_.y() += direction.y() * step;

  double force_error = collection_f0_ - std::abs(filtered_force_z_ - baseline_force_z_);
  force_error_integral_ += force_error / 1000.0;
  force_error_integral_ = std::max(std::min(force_error_integral_, 0.05), -0.05);
  z_offset_ = force_kp_ * force_error + force_ki_ * force_error_integral_;
  position_d_target_.z() = slide_base_z_ + z_offset_;

  orientation_d_target_ = teach_start_orientation_;

  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  if (update_count_ % 500 == 0) {
    auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
    Eigen::Vector3d pos = transform.translation();

    RCLCPP_INFO(get_node()->get_logger(),
                "[SLIDE] pos=[%.4f, %.4f, %.4f] target_xy=[%.4f, %.4f] "
                "force_z=%.3f z_offset=%.4f dist=%.4f",
                pos.x(), pos.y(), pos.z(),
                target_xy.x(), target_xy.y(),
                filtered_force_z_, z_offset_, distance);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateLift() {
  position_d_target_.z() = slide_base_z_ + approach_height_;
  orientation_d_target_ = teach_start_orientation_;

  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(pose_matrix.data()));
  Eigen::Vector3d current_pos = transform.translation();

  if (std::abs(current_pos.z() - (slide_base_z_ + approach_height_)) < 0.005) {
    phase_ = Phase::WAIT_PARAMS;
    collection_params_received_ = false;
    force_error_integral_ = 0.0;
    z_offset_ = 0.0;

    std_msgs::msg::Bool completion_msg;
    completion_msg.data = true;
    completion_publisher_->publish(completion_msg);

    RCLCPP_INFO(get_node()->get_logger(),
                "LIFT -> WAIT_PARAMS (one collection cycle complete)");
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateWaitParams() {
  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }

  if (collection_params_received_) {
    collection_params_received_ = false;
    phase_ = Phase::APPROACH;
    setStiffnessForPhase(Phase::APPROACH);
    initImpedanceState();

    RCLCPP_INFO(get_node()->get_logger(),
                "WAIT_PARAMS -> APPROACH (dx=%.4f dy=%.4f dx_step=%.4f dy_step=%.4f F0=%.2f)",
                collection_dx_, collection_dy_, collection_dx_step_,
                collection_dy_step_, collection_f0_);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type DataCollectionController::updateDone() {
  Eigen::Matrix<double, 7, 1> tau_d;
  computeImpedanceControl(tau_d);
  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    last_cmd_tau_[i] = tau_d(i);
  }
  return controller_interface::return_type::OK;
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::DataCollectionController,
                       controller_interface::ControllerInterface)
