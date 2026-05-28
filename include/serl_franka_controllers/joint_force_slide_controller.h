#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <franka/robot_state.h>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <serl_franka_controllers/pseudo_inversion.h>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_controllers {

class JointForceSlideController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  enum class Phase { JOINT_MOVE = 0, SETTLE = 1, DESCEND = 2, SLIDE = 3 };

  void updateJointStates();
  franka::RobotState* getRobotStatePtr();
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);
  void computeImpedanceControl(Eigen::Matrix<double, 7, 1>& tau_d);
  bool checkSafetyLimits();
  void initImpedanceState();
  void setStiffnessForPhase(Phase phase);
  Phase parsePhaseString(const std::string& phase_str);
  std::string phaseToString(Phase p);

  controller_interface::return_type updateJointMove(const rclcpp::Duration& period);
  controller_interface::return_type updateSettle();
  controller_interface::return_type updateDescend();
  controller_interface::return_type updateSlide();

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  std::string arm_id_;
  static constexpr int num_joints_ = 7;

  std::array<double, 7> q_{};
  std::array<double, 7> dq_{};
  std::array<double, 7> tau_J_d_last_{};
  std::array<double, 7> last_cmd_tau_{};
  franka::RobotState* robot_state_ptr_ = nullptr;

  std::array<double, 7> initial_joint_pose_{};
  std::array<double, 7> target_joint_pose_{};
  std::array<double, 7> k_gains_{};
  std::array<double, 7> d_gains_{};
  double motion_duration_ = 10.0;
  rclcpp::Duration joint_elapsed_time_ = rclcpp::Duration(0, 0);

  Phase phase_ = Phase::JOINT_MOVE;
  std::string start_phase_str_ = "joint_move";
  int settle_count_ = 0;
  int settle_cycles_ = 1000;
  double baseline_force_z_ = 0.0;
  double filtered_force_z_ = 0.0;

  double target_force_ = 1.0;
  double descend_speed_ = 0.005;
  double slide_speed_ = 0.01;
  double slide_distance_ = 0.1;
  double force_kp_ = 0.005;
  double force_ki_ = 0.01;
  double force_filter_alpha_ = 0.1;
  double max_descend_distance_ = 0.15;
  double force_error_integral_ = 0.0;
  double z_offset_ = 0.0;
  double slide_start_x_ = 0.0;
  double descend_start_z_ = 0.0;
  double slide_base_z_ = 0.0;

  double filter_params_ = 0.005;
  double position_filter_ = 0.005;
  double nullspace_stiffness_ = 20.0;
  double nullspace_stiffness_target_ = 20.0;
  double joint1_nullspace_stiffness_ = 20.0;
  double joint1_nullspace_stiffness_target_ = 20.0;
  const double delta_tau_max_{1.0};

  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;

  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Matrix<double, 6, 1> error_;
  Eigen::Matrix<double, 6, 1> error_i_;

  double translational_stiffness_xy_ = 2000.0;
  double translational_stiffness_z_settle_ = 2000.0;
  double translational_stiffness_z_descend_ = 1500.0;
  double translational_stiffness_z_slide_ = 200.0;
  double rotational_stiffness_ = 150.0;
  double translational_damping_xy_ = 89.0;
  double translational_damping_z_settle_ = 89.0;
  double translational_damping_z_descend_ = 60.0;
  double translational_damping_z_slide_ = 30.0;
  double rotational_damping_ = 7.0;

  double max_joint_velocity_ = 2.0;
  double max_cartesian_velocity_ = 0.5;
  double max_torque_ = 30.0;
  bool safety_violation_ = false;

  int robot_state_interface_index_ = -1;

  double max_update_time_us_ = 0.0;
  double last_update_time_us_ = 0.0;
  int update_count_ = 0;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> phase_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> completion_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> filtered_force_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> diagnostics_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> timing_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> command_torques_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> force_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_subscriber_;
  void resetCallback(const std_msgs::msg::Bool::SharedPtr msg);
};

}  // namespace serl_franka_controllers
