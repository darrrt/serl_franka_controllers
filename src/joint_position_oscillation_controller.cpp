#include <serl_franka_controllers/joint_position_oscillation_controller.h>

#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace serl_franka_controllers {

controller_interface::InterfaceConfiguration
JointPositionOscillationController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionOscillationController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  return config;
}

CallbackReturn JointPositionOscillationController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<double>("motion_duration", 10.0);
    auto_declare<std::vector<double>>("target_joint_positions_a", {});
    auto_declare<std::vector<double>>("target_joint_positions_b", {});
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionOscillationController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  motion_duration_ = get_node()->get_parameter("motion_duration").as_double();

  auto target_a = get_node()->get_parameter("target_joint_positions_a").as_double_array();
  if (target_a.size() != static_cast<size_t>(num_joints_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointPositionOscillationController: target_joint_positions_a size is %zu, expected %d",
                 target_a.size(), num_joints_);
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < target_a.size(); ++i) {
    target_pose_a_[i] = target_a[i];
  }

  auto target_b = get_node()->get_parameter("target_joint_positions_b").as_double_array();
  if (target_b.size() != static_cast<size_t>(num_joints_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointPositionOscillationController: target_joint_positions_b size is %zu, expected %d",
                 target_b.size(), num_joints_);
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < target_b.size(); ++i) {
    target_pose_b_[i] = target_b[i];
  }

  auto k = get_node()->get_parameter("k_gains").as_double_array();
  auto d = get_node()->get_parameter("d_gains").as_double_array();
  if (k.size() == 7 && d.size() == 7) {
    for (size_t i = 0; i < 7; ++i) {
      k_gains_[i] = k[i];
      d_gains_[i] = d[i];
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionOscillationController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    initial_pose_[i] = state_interfaces_[i].get_value();
  }
  elapsed_time_ = rclcpp::Duration(0, 0);
  moving_to_b_ = true;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPositionOscillationController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;
  double t = elapsed_time_.seconds() / motion_duration_;

  if (t >= 1.0) {
    for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
      initial_pose_[i] = state_interfaces_[i].get_value();
    }
    moving_to_b_ = !moving_to_b_;
    elapsed_time_ = rclcpp::Duration(0, 0);
    t = 0.0;
  }

  const auto& target = moving_to_b_ ? target_pose_b_ : target_pose_a_;

  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    double q = state_interfaces_[i].get_value();
    double dq = state_interfaces_[num_joints_ + i].get_value();

    double s = 6.0 * std::pow(t, 5) - 15.0 * std::pow(t, 4) + 10.0 * std::pow(t, 3);

    double q_d = initial_pose_[i] + s * (target[i] - initial_pose_[i]);
    double dq_d = (30.0 * std::pow(t, 4) - 60.0 * std::pow(t, 3) + 30.0 * std::pow(t, 2)) *
                  (target[i] - initial_pose_[i]) / motion_duration_;

    double tau = k_gains_[i] * (q_d - q) + d_gains_[i] * (dq_d - dq);
    command_interfaces_[i].set_value(tau);
  }

  return controller_interface::return_type::OK;
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointPositionOscillationController,
                       controller_interface::ControllerInterface)
