#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <Eigen/Dense>

#include <franka/robot_state.h>
#include <franka_semantic_components/franka_robot_model.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <serl_franka_controllers/pseudo_inversion.h>

#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/move.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace serl_franka_controllers {

class DataCollectionController : public controller_interface::ControllerInterface {
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
  enum class Phase {
    GRASP = 0,
    TEACH = 1,
    CALIBRATE = 2,
    APPROACH = 3,
    DESCEND = 4,
    SLIDE = 5,
    LIFT = 6,
    WAIT_PARAMS = 7,
    DONE = 8
  };

  enum class CalibSubPhase {
    RISE = 0,
    MOVE_TO_CONFIG = 1,
    SETTLE = 2,
    RECORD = 3,
    SOLVE = 4,
    RETURN = 5,
    FINISHED = 6
  };

  enum class GraspState {
    IDLE = 0,
    SENDING = 1,
    WAITING = 2,
    SUCCEEDED = 3,
    FAILED = 4
  };

  void updateJointStates();
  franka::RobotState* getRobotStatePtr();
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);
  void computeImpedanceControl(Eigen::Matrix<double, 7, 1>& tau_d);
  void computeGravityCompensation(Eigen::Matrix<double, 7, 1>& tau_d);
  bool checkSafetyLimits();
  void initImpedanceState();
  void setStiffnessForPhase(Phase phase);
  std::string phaseToString(Phase p);

  controller_interface::return_type updateGrasp();
  controller_interface::return_type updateTeach();
  controller_interface::return_type updateCalibrate();
  controller_interface::return_type updateApproach();
  controller_interface::return_type updateDescend();
  controller_interface::return_type updateSlide();
  controller_interface::return_type updateLift();
  controller_interface::return_type updateWaitParams();
  controller_interface::return_type updateDone();

  void sendGripperGrasp();
  void sendGripperOpen();
  void gripperTimerCallback();
  void graspGoalResponseCallback(
      const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::SharedPtr& goal_handle);
  void graspResultCallback(
      const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult& result);
  void moveGoalResponseCallback(
      const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::SharedPtr& goal_handle);
  void moveResultCallback(
      const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result);

  void buildGravityRegressor(const Eigen::Matrix3d& R_EE,
                             Eigen::Matrix<double, 6, 4>& Y);
  void solvePayloadCalibration();

  void teachTriggerCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void collectionParamsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void resetCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  std::string arm_id_;
  static constexpr int num_joints_ = 7;

  std::array<double, 7> q_{};
  std::array<double, 7> dq_{};
  std::array<double, 7> tau_J_d_last_{};
  std::array<double, 7> last_cmd_tau_{};
  franka::RobotState* robot_state_ptr_ = nullptr;

  Phase phase_ = Phase::GRASP;
  int update_count_ = 0;

  double filter_params_ = 0.005;
  double position_filter_ = 0.005;
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

  double nullspace_stiffness_ = 20.0;
  double nullspace_stiffness_target_ = 20.0;
  double joint1_nullspace_stiffness_ = 20.0;
  double joint1_nullspace_stiffness_target_ = 20.0;

  double translational_stiffness_xy_ = 2000.0;
  double translational_stiffness_z_ = 1500.0;
  double rotational_stiffness_ = 150.0;
  double translational_damping_xy_ = 89.0;
  double translational_damping_z_ = 60.0;
  double rotational_damping_ = 7.0;

  double max_joint_velocity_ = 2.0;
  double max_cartesian_velocity_ = 0.5;
  double max_torque_ = 30.0;
  bool safety_violation_ = false;

  int robot_state_interface_index_ = -1;

  double grasp_width_ = 0.0;
  double grasp_force_ = 50.0;
  double grasp_speed_ = 0.1;
  double grasp_epsilon_inner_ = 0.005;
  double grasp_epsilon_outer_ = 0.005;
  int max_grasp_attempts_ = 3;
  int grasp_attempt_count_ = 0;
  std::atomic<GraspState> grasp_state_{GraspState::IDLE};
  std::atomic<bool> grasp_request_{false};
  std::atomic<bool> open_request_{false};
  bool grasp_succeeded_ = false;
  std::string gripper_action_prefix_;

  double teach_damping_translational_ = 10.0;
  double teach_damping_rotational_ = 1.0;
  bool teach_triggered_ = false;
  Eigen::Vector3d teach_start_position_;
  Eigen::Quaterniond teach_start_orientation_;
  double slide_base_z_ = 0.0;

  double calib_rise_height_ = 0.15;
  double calib_settle_cycles_ = 1000;
  int calib_settle_count_ = 0;
  CalibSubPhase calib_sub_phase_ = CalibSubPhase::RISE;
  int calib_current_config_ = 0;
  Eigen::Vector3d calib_rise_position_;
  Eigen::Quaterniond calib_rise_orientation_;
  std::vector<Eigen::Quaterniond> calib_orientations_;
  Eigen::VectorXd calib_W_stack_;
  Eigen::MatrixXd calib_Y_stack_;
  int calib_record_count_ = 0;
  double payload_mass_ = 0.0;
  Eigen::Vector3d payload_com_ = Eigen::Vector3d::Zero();

  double default_dx_ = 0.01;
  double default_dy_ = 0.0;
  double default_dx_step_ = 0.01;
  double default_dy_step_ = 0.0;
  double default_f0_ = 1.0;

  double collection_dx_ = 0.01;
  double collection_dy_ = 0.0;
  double collection_dx_step_ = 0.01;
  double collection_dy_step_ = 0.0;
  double collection_f0_ = 1.0;
  bool collection_params_received_ = false;

  double target_force_ = 1.0;
  double descend_speed_ = 0.005;
  double slide_speed_ = 0.01;
  double force_kp_ = 0.005;
  double force_ki_ = 0.01;
  double force_filter_alpha_ = 0.1;
  double max_descend_distance_ = 0.25;
  double force_error_integral_ = 0.0;
  double z_offset_ = 0.0;
  double filtered_force_z_ = 0.0;
  double baseline_force_z_ = 0.0;
  double descend_start_z_ = 0.0;

  double approach_height_ = 0.05;

  Eigen::Vector3d slide_start_xy_;
  Eigen::Vector3d slide_target_xy_;
  double slide_target_z_ = 0.0;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> phase_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> completion_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> filtered_force_publisher_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> diagnostics_publisher_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>> force_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> payload_mass_publisher_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64MultiArray>> payload_com_publisher_;

  rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr grasp_client_;
  rclcpp_action::Client<franka_msgs::action::Move>::SharedPtr move_client_;
  rclcpp::TimerBase::SharedPtr gripper_timer_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teach_trigger_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr collection_params_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_subscriber_;
};

}  // namespace serl_franka_controllers
