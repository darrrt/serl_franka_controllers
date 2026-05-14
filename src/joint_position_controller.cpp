#include <serl_franka_controllers/joint_position_controller.h>

#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace serl_franka_controllers {

controller_interface::InterfaceConfiguration
JointPositionController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointPositionController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }

  return config;
}

CallbackReturn JointPositionController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<std::vector<double>>("target_joint_positions", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  auto target_positions = get_node()->get_parameter("target_joint_positions").as_double_array();
  if (target_positions.size() != static_cast<size_t>(num_joints_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "JointPositionController: target_joint_positions size is %zu, expected %d",
                 target_positions.size(), num_joints_);
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < target_positions.size(); ++i) {
    reset_pose_[i] = target_positions[i];
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    initial_pose_[i] = state_interfaces_[i].get_value();
  }
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointPositionController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;

  for (size_t i = 0; i < static_cast<size_t>(num_joints_); ++i) {
    if (elapsed_time_.seconds() > 10) {
      command_interfaces_[i].set_value(reset_pose_[i]);
    } else {
      double t = elapsed_time_.seconds() / 10.0;
      double pos = (1.0 - t) * initial_pose_[i] + t * reset_pose_[i];
      command_interfaces_[i].set_value(pos);
    }
  }

  return controller_interface::return_type::OK;
}

double JointPositionController::cubicInterpolation(double p0, double p1, double t) {
  return p0 + (p1 - p0) * t;
}

}  // namespace serl_franka_controllers

PLUGINLIB_EXPORT_CLASS(serl_franka_controllers::JointPositionController,
                       controller_interface::ControllerInterface)
