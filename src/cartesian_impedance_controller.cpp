#include <serl_franka_controllers/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>
#include <string>

#include <hardware_interface/loaned_command_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka/robot_state.h>
#include <serl_franka_controllers/pseudo_inversion.h>

namespace serl_franka_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    state_interfaces_config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    state_interfaces_config.names.push_back(franka_robot_model_name);
  }

  return state_interfaces_config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      arm_id_ + "/robot_model", arm_id_ + "/robot_state");

  publisher_franka_jacobian_ =
      get_node()->create_publisher<serl_franka_controllers::msg::ZeroJacobian>(
          "franka_jacobian", rclcpp::SystemDefaultsQoS());

  sub_equilibrium_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "equilibrium_pose", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        equilibriumPoseCallback(msg);
      });

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  translational_clip_min_.setConstant(-std::numeric_limits<double>::max());
  translational_clip_max_.setConstant(std::numeric_limits<double>::max());
  rotational_clip_min_.setConstant(-std::numeric_limits<double>::max());
  rotational_clip_max_.setConstant(std::numeric_limits<double>::max());
  // 控制器目前没有位置限制 ，仅靠硬件保护

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  updateJointStates();

  auto pose_matrix = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(pose_matrix.data()));

  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(q_.data());
  q_d_nullspace_ = q_initial;

  error_i.setZero();

  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_.topLeftCorner(3, 3) << 2000.0 * Eigen::Matrix3d::Identity();
  cartesian_stiffness_.bottomRightCorner(3, 3) << 150.0 * Eigen::Matrix3d::Identity();
  cartesian_damping_.setIdentity();
  cartesian_damping_.topLeftCorner(3, 3) << 89.0 * Eigen::Matrix3d::Identity();
  cartesian_damping_.bottomRightCorner(3, 3) << 7.0 * Eigen::Matrix3d::Identity();

  cartesian_stiffness_target_ = cartesian_stiffness_;
  cartesian_damping_target_ = cartesian_damping_;

  tau_J_d_last_.fill(0.0);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->release_interfaces();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints_; ++i) {
    q_.at(i) = state_interfaces_[i].get_value();
    dq_.at(i) = state_interfaces_[num_joints_ + i].get_value();
  }
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();

  std::array<double, 42> jacobian_array =
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  std::array<double, 16> o_t_ee_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  Eigen::Affine3d transform(Eigen::Matrix4d::Map(o_t_ee_array.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());

  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_.data());

  error_.head(3) << position - position_d_;
  for (int i = 0; i < 3; i++) {
    error_(i) =
        std::min(std::max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
  }

  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  error_.tail(3) << -transform.linear() * error_.tail(3);
  for (int i = 0; i < 3; i++) {
    error_(i + 3) =
        std::min(std::max(error_(i + 3), rotational_clip_min_(i)), rotational_clip_max_(i));
  }

  error_i.head(3) << (error_i.head(3) + error_.head(3)).cwiseMax(-0.1).cwiseMin(0.1);
  error_i.tail(3) << (error_i.tail(3) + error_.tail(3)).cwiseMax(-0.3).cwiseMin(0.3);

  Eigen::Matrix<double, 7, 1> tau_task, tau_nullspace, tau_d;

  Eigen::Matrix<double, 6, 7> jacobian_transpose_pinv;
  Eigen::Matrix<double, 7, 6> jacobian_T = jacobian.transpose();
  pseudoInverse(jacobian_T, jacobian_transpose_pinv);

  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error_ - cartesian_damping_ * (jacobian * dq) -
                   Ki_ * error_i);

  Eigen::Matrix<double, 7, 1> dqe;
  Eigen::Matrix<double, 7, 1> qe;

  qe << q_d_nullspace_ - q;
  qe.head(1) << qe.head(1) * joint1_nullspace_stiffness_;
  dqe << dq;
  dqe.head(1) << dqe.head(1) * 2.0 * sqrt(joint1_nullspace_stiffness_);
  tau_nullspace << (Eigen::Matrix<double, 7, 7>::Identity() -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * qe - (2.0 * sqrt(nullspace_stiffness_)) * dqe);

  tau_d << tau_task + tau_nullspace + coriolis;

  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d_last(tau_J_d_last_.data());
  tau_d << saturateTorqueRate(tau_d, tau_J_d_last);

  for (size_t i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
    tau_J_d_last_[i] = tau_d(i);
  }

  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  joint1_nullspace_stiffness_ =
      filter_params_ * joint1_nullspace_stiffness_target_ +
      (1.0 - filter_params_) * joint1_nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  Ki_ = filter_params_ * Ki_target_ + (1.0 - filter_params_) * Ki_;

  return controller_interface::return_type::OK;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
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

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  error_i.setZero();
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)