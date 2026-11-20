#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"


#include <phidget22.h>

namespace phidgets_hardware
{

/*
 * Threading model
 * ---------------
 *  - Control thread (controller_manager): read() / write().
 *  - Telemetry thread: kill / power commands from /rover/relay_board/command.
 *  - Reconnect worker thread: slow (blocking) Phidget attachment.
 *
 * hw_mutex_ guards every Phidget handle and every per-motor vector below.
 * It is only ever held briefly: the slow Phidget_openWaitForAttachment()
 * calls run WITHOUT the lock, so a kill is never stuck behind a reconnect.
 */
class PhidgetsBldcHardware : public hardware_interface::SystemInterface
{
public:


  RCLCPP_SHARED_PTR_DEFINITIONS(PhidgetsBldcHardware)
  ~PhidgetsBldcHardware();
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  enum class FailsafeState { NOT_ENABLED, ENABLED, UNSUPPORTED };

  // Dynamic configuration (auto-detected from URDF). Immutable after on_init.
  std::vector<int32_t> device_serial_;
  std::vector<int> hub_port_;
  std::vector<int> channel_;
  std::vector<double> direction_sign_;
  std::vector<std::string> joint_names_;
  std::vector<bool> is_left_wheel_;
  std::vector<bool> is_middle_wheel_;

  // ---- everything below until "Configuration" is guarded by hw_mutex_ ----
  std::mutex hw_mutex_;

  // Motor state tracking
  std::vector<bool> motor_enabled_;
  std::vector<bool> attached_;
  std::vector<bool> temperature_attached_;
  std::vector<FailsafeState> failsafe_state_;
  std::vector<std::chrono::steady_clock::time_point> last_failsafe_reset_;

  // Phidget handles
  std::vector<PhidgetBLDCMotorHandle> motors_;
  std::vector<PhidgetTemperatureSensorHandle> temperature_sensors_;

  // Data storage (state/command interfaces point into these)
  std::vector<double> pos_rad_;
  std::vector<double> vel_state_;
  std::vector<double> temperature_c_;
  std::vector<double> cmd_;

  // Configuration parameters
  double command_limit_{1.0};
  double acceleration_{5.0};
  double stall_velocity_{0.0};
  double gear_ratio_{106.0};
  int commutations_per_motor_rev_{24};
  double rescale_factor_rot_{1.0};
  double corner_steering_scale_{1.0};
  double middle_steering_scale_{0.55};
  // Phidget on-board watchdog: if write() stops resetting it for this long
  // (process frozen/crashed, USB stalled), the controller stops the motor by
  // itself and refuses commands until the channel is reopened. 0 = off.
  uint32_t failsafe_timeout_ms_{500};

  // Safety / lifecycle flags (lock-free, read from any thread)
  std::atomic<bool> software_kill_active_{false};
  std::atomic<bool> reconnect_in_progress_{false};
  std::atomic<bool> shutting_down_{false};

  // Reconnect worker
  std::thread reconnect_thread_;
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;   // interrupts retry sleeps on kill/shutdown

  // ROS Communication
  rclcpp::Node::SharedPtr telemetry_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr telemetry_executor_;
  std::thread telemetry_thread_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_telemetry_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr kill_sub_;
  rclcpp::Time last_telemetry_publish_time_{0, 0, RCL_ROS_TIME};
  double telemetry_publish_period_sec_{0.2};

  static double clamp(double x, double lo, double hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  void setup_ros_communication();
  void software_kill_callback(const std_msgs::msg::String::SharedPtr msg);

  // Kill / power handling (called from the telemetry thread)
  void activate_software_kill(const std::string & reason);
  void request_reconnect(const std::vector<size_t> & indices, const std::string & reason);

  // Reconnect worker (blocking; never holds hw_mutex_ while waiting on hardware)
  void reconnect_worker(std::vector<size_t> indices);
  bool connect_motor(size_t i);            // returns false if aborted by kill/shutdown
  bool connection_aborted() const;
  bool interruptible_sleep(std::chrono::milliseconds duration);
  PhidgetBLDCMotorHandle open_motor(size_t i);
  PhidgetTemperatureSensorHandle open_temperature_sensor(size_t i);

  // Require hw_mutex_ held
  void publish_motor_telemetry();
  void disable_motor(size_t i, const std::string & reason);
  void close_phidget(size_t i);
  void close_temperature_sensor(size_t i);
  void service_failsafe(size_t i);
  bool both_middle_wheels_attached() const;
  void log_connection_summary();

  bool is_left_wheel_joint(const std::string & joint_name);
  bool is_middle_wheel_joint(const std::string & joint_name);
  int find_joint(const std::string & name) const;
  double axle_weighted_command(size_t i, double left, double right) const;
};

}  // namespace phidgets_hardware
