#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <Eigen/Dense>

#include <franka_semantic_components/franka_robot_model.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <serl_franka_controllers/msg/zero_jacobian.hpp>

namespace serl_franka_controllers {

class CartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  void updateJointStates();
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  std::string arm_id_;
  static constexpr int num_joints_ = 7;

  std::array<double, 7> q_{};
  std::array<double, 7> dq_{};
  std::array<double, 7> tau_J_d_last_{};

  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  double joint1_nullspace_stiffness_{20.0};
  double joint1_nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> Ki_;
  Eigen::Matrix<double, 6, 6> Ki_target_;

  Eigen::Matrix<double, 3, 1> translational_clip_min_;
  Eigen::Matrix<double, 3, 1> translational_clip_max_;
  Eigen::Matrix<double, 3, 1> rotational_clip_min_;
  Eigen::Matrix<double, 3, 1> rotational_clip_max_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Matrix<double, 6, 1> error_i;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;

  std::shared_ptr<rclcpp::Publisher<serl_franka_controllers::msg::ZeroJacobian>>
      publisher_franka_jacobian_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

}  // namespace serl_franka_controllers
