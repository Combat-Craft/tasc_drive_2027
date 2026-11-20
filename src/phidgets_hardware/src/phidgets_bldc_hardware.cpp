#include "phidgets_hardware/phidgets_bldc_hardware.hpp"
#include <cmath>
#include <numeric>
#include <iomanip>
#include <sstream>
#include <pluginlib/class_list_macros.hpp>
#include <nlohmann/json.hpp> 

namespace phidgets_hardware
{

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & key, const std::string & def) -> std::string {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  gear_ratio_ = std::stod(get_param("gear_ratio", "106.0"));
  commutations_per_motor_rev_ = std::stoi(get_param("commutations_per_motor_rev", "24"));
  acceleration_ = std::stod(get_param("acceleration", "5.0"));
  stall_velocity_ = std::stod(get_param("stall_velocity", "0.0"));
  command_limit_ = std::stod(get_param("command_limit", "1.0"));
  corner_steering_scale_ = clamp(std::stod(get_param("corner_steering_scale", "1.0")), 0.0, 1.0);
  middle_steering_scale_ = clamp(std::stod(get_param("middle_steering_scale", "0.55")), 0.0, 1.0);
  failsafe_timeout_ms_ = static_cast<uint32_t>(
    std::max(0, std::stoi(get_param("failsafe_timeout_ms", "500"))));

  rescale_factor_rot_ = 1.0 / (gear_ratio_ * static_cast<double>(commutations_per_motor_rev_));

  // Auto-detect motor configuration from URDF joint names
  int32_t default_serial = 765818;
  
  for (const auto& joint : info_.joints) {
    const std::string& name = joint.name;
    
    // Skip if not a wheel joint
    if (name.find("wheel") == std::string::npos) continue;
    
    joint_names_.push_back(name);
    is_left_wheel_.push_back(is_left_wheel_joint(name));
    is_middle_wheel_.push_back(is_middle_wheel_joint(name));
    
    // Map based on naming convention
    if (name.find("front_left") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(0);
      channel_.push_back(0);
      direction_sign_.push_back(-1.0);
    } else if (name.find("middle_left") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(1);
      channel_.push_back(0);
      direction_sign_.push_back(-1.0);
    } else if (name.find("rear_left") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(2);
      channel_.push_back(0);
      direction_sign_.push_back(-1.0);
    } else if (name.find("front_right") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(5);
      channel_.push_back(0);
      direction_sign_.push_back(1.0);
    } else if (name.find("middle_right") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(4);
      channel_.push_back(0);
      direction_sign_.push_back(1.0);
    } else if (name.find("rear_right") != std::string::npos) {
      device_serial_.push_back(default_serial);
      hub_port_.push_back(3);
      channel_.push_back(0);
      direction_sign_.push_back(1.0);
    } else {
      // Generic assignment
      static int generic_counter = 0;
      device_serial_.push_back(default_serial);
      hub_port_.push_back(generic_counter++ % 6);
      channel_.push_back(0);
      direction_sign_.push_back(is_left_wheel_.back() ? -1.0 : 1.0);
    }
  }
  
  const size_t num_motors = joint_names_.size();
  
  if (num_motors == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"), "No wheel joints found!");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Initialize all vectors
  motors_.resize(num_motors, nullptr);
  temperature_sensors_.resize(num_motors, nullptr);
  motor_enabled_.resize(num_motors, false);
  failsafe_state_.resize(num_motors, FailsafeState::NOT_ENABLED);
  last_failsafe_reset_.resize(num_motors);
  attached_.resize(num_motors, false);
  temperature_attached_.resize(num_motors, false);
  pos_rad_.resize(num_motors, 0.0);
  vel_state_.resize(num_motors, 0.0);
  temperature_c_.resize(num_motors, 0.0);
  cmd_.resize(num_motors, 0.0);
  
  setup_ros_communication();
  
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Auto-detected %zu motors. Connecting all motors automatically...", num_motors);
  
  if (failsafe_timeout_ms_ == 0) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
                "Phidget failsafe DISABLED (failsafe_timeout_ms = 0)");
  }

  // Initial connection runs synchronously (the control loop is not running yet).
  std::vector<size_t> all(num_motors);
  std::iota(all.begin(), all.end(), 0);
  reconnect_in_progress_ = true;
  reconnect_worker(all);
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool PhidgetsBldcHardware::is_left_wheel_joint(const std::string& joint_name)
{
  return joint_name.find("left") != std::string::npos;
}

bool PhidgetsBldcHardware::is_middle_wheel_joint(const std::string& joint_name)
{
  return joint_name.find("middle") != std::string::npos;
}

bool PhidgetsBldcHardware::both_middle_wheels_attached() const
{
  bool left_middle_attached = false;
  bool right_middle_attached = false;

  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (!is_middle_wheel_[i] || !motor_enabled_[i] || !attached_[i]) {
      continue;
    }

    if (is_left_wheel_[i]) {
      left_middle_attached = true;
    } else {
      right_middle_attached = true;
    }
  }

  return left_middle_attached && right_middle_attached;
}

double PhidgetsBldcHardware::axle_weighted_command(size_t i, double left, double right) const
{
  const double throttle_component = (left + right) / 2.0;
  const double steering_component = (left - right) / 2.0;
  const double steering_scale = is_middle_wheel_[i] ? middle_steering_scale_ : corner_steering_scale_;
  const double weighted_steering = steering_component * steering_scale;

  if (is_left_wheel_[i]) {
    return throttle_component + weighted_steering;
  }
  return throttle_component - weighted_steering;
}



int PhidgetsBldcHardware::find_joint(const std::string & name) const
{
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (joint_names_[i] == name) return static_cast<int>(i);
  }
  return -1;
}

// ---------------------------------------------------------------------------
// Connection handling
// ---------------------------------------------------------------------------

bool PhidgetsBldcHardware::connection_aborted() const
{
  return software_kill_active_.load() || shutting_down_.load();
}

bool PhidgetsBldcHardware::interruptible_sleep(std::chrono::milliseconds duration)
{
  std::unique_lock<std::mutex> lk(wake_mutex_);
  wake_cv_.wait_for(lk, duration, [this]() { return connection_aborted(); });
  return !connection_aborted();
}

PhidgetBLDCMotorHandle PhidgetsBldcHardware::open_motor(size_t i)
{
  // Touches only immutable config, so it is safe to call without hw_mutex_.
  PhidgetBLDCMotorHandle h = nullptr;
  if (PhidgetBLDCMotor_create(&h) != EPHIDGET_OK || h == nullptr) return nullptr;

  Phidget_setDeviceSerialNumber((PhidgetHandle)h, device_serial_[i]);
  Phidget_setHubPort((PhidgetHandle)h, hub_port_[i]);
  Phidget_setChannel((PhidgetHandle)h, channel_[i]);

  if (Phidget_openWaitForAttachment((PhidgetHandle)h, 3000) != EPHIDGET_OK) {
    Phidget_close((PhidgetHandle)h);
    PhidgetBLDCMotor_delete(&h);
    return nullptr;
  }

  PhidgetBLDCMotor_setRescaleFactor(h, rescale_factor_rot_);
  PhidgetBLDCMotor_setAcceleration(h, acceleration_);
  PhidgetBLDCMotor_setStallVelocity(h, stall_velocity_);
  PhidgetBLDCMotor_setTargetVelocity(h, 0.0);
  return h;
}

PhidgetTemperatureSensorHandle PhidgetsBldcHardware::open_temperature_sensor(size_t i)
{
  PhidgetTemperatureSensorHandle h = nullptr;
  if (PhidgetTemperatureSensor_create(&h) != EPHIDGET_OK || h == nullptr) return nullptr;

  Phidget_setDeviceSerialNumber((PhidgetHandle)h, device_serial_[i]);
  Phidget_setHubPort((PhidgetHandle)h, hub_port_[i]);
  Phidget_setChannel((PhidgetHandle)h, channel_[i]);

  if (Phidget_openWaitForAttachment((PhidgetHandle)h, 500) != EPHIDGET_OK) {
    Phidget_close((PhidgetHandle)h);
    PhidgetTemperatureSensor_delete(&h);
    return nullptr;
  }
  return h;
}

bool PhidgetsBldcHardware::connect_motor(size_t i)
{
  auto logger = rclcpp::get_logger("PhidgetsBldcHardware");
  const int max_attempts = 5;

  RCLCPP_INFO(logger, "Connecting to motor %zu: %s...", i, joint_names_[i].c_str());

  for (int attempt = 1; attempt <= max_attempts; attempt++) {
    if (connection_aborted() || !rclcpp::ok()) return false;

    // Slow part, done WITHOUT the lock so kill/read/write are never blocked.
    PhidgetBLDCMotorHandle motor = open_motor(i);

    if (motor) {
      PhidgetTemperatureSensorHandle temp = open_temperature_sensor(i);

      std::lock_guard<std::mutex> lk(hw_mutex_);
      if (connection_aborted()) {
        // A kill arrived while we were attaching: do not install the handle.
        PhidgetBLDCMotor_setTargetVelocity(motor, 0.0);
        Phidget_close((PhidgetHandle)motor);
        PhidgetBLDCMotor_delete(&motor);
        if (temp) {
          Phidget_close((PhidgetHandle)temp);
          PhidgetTemperatureSensor_delete(&temp);
        }
        return false;
      }

      motors_[i] = motor;
      temperature_sensors_[i] = temp;
      attached_[i] = true;
      temperature_attached_[i] = (temp != nullptr);
      motor_enabled_[i] = true;
      failsafe_state_[i] = FailsafeState::NOT_ENABLED;  // armed on first write()
      cmd_[i] = 0.0;

      RCLCPP_INFO(logger, "Motor %zu (%s) CONNECTED (attempt %d)",
                  i, joint_names_[i].c_str(), attempt);
      return true;
    }

    RCLCPP_WARN(logger, "Motor %zu (%s) failed (attempt %d/%d). Retrying in 3s...",
                i, joint_names_[i].c_str(), attempt, max_attempts);

    if (attempt < max_attempts && !interruptible_sleep(std::chrono::seconds(3))) {
      return false;
    }
  }

  RCLCPP_ERROR(logger, "Motor %zu (%s) FAILED after %d attempts",
               i, joint_names_[i].c_str(), max_attempts);
  return true;  // not aborted, just failed; keep going with the other motors
}

void PhidgetsBldcHardware::reconnect_worker(std::vector<size_t> indices)
{
  auto logger = rclcpp::get_logger("PhidgetsBldcHardware");

  {
    // Full reset of the requested channels (same as boot).
    std::lock_guard<std::mutex> lk(hw_mutex_);
    for (size_t i : indices) {
      if (motors_[i]) PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
      close_phidget(i);
      close_temperature_sensor(i);
      attached_[i] = false;
      temperature_attached_[i] = false;
      motor_enabled_[i] = false;
      failsafe_state_[i] = FailsafeState::NOT_ENABLED;
      pos_rad_[i] = 0.0;
      vel_state_[i] = 0.0;
      cmd_[i] = 0.0;
    }
  }

  bool aborted = false;
  for (size_t i : indices) {
    if (!connect_motor(i)) {
      aborted = true;
      break;
    }
  }

  if (aborted) {
    RCLCPP_WARN(logger, "Motor connection ABORTED (software kill or shutdown)");
  }

  {
    std::lock_guard<std::mutex> lk(hw_mutex_);
    log_connection_summary();
  }
  reconnect_in_progress_ = false;
}

void PhidgetsBldcHardware::request_reconnect(
  const std::vector<size_t> & indices, const std::string & reason)
{
  auto logger = rclcpp::get_logger("PhidgetsBldcHardware");
  if (shutting_down_ || indices.empty()) return;

  bool expected = false;
  if (!reconnect_in_progress_.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(logger, "%s ignored: a motor connection is already in progress",
                reason.c_str());
    return;
  }

  {
    // Repeated turn-on requests (the dashboard sends one per second) must not
    // tear down motors that are already running fine.
    std::lock_guard<std::mutex> lk(hw_mutex_);
    bool all_ok = true;
    for (size_t i : indices) {
      all_ok = all_ok && motor_enabled_[i] && attached_[i] && motors_[i];
    }
    if (all_ok) {
      RCLCPP_INFO(logger, "%s: motors already connected, nothing to do", reason.c_str());
      reconnect_in_progress_ = false;
      return;
    }
  }

  RCLCPP_WARN(logger, "%s - reconnecting %zu motor(s) in the background",
              reason.c_str(), indices.size());

  // Only this (telemetry) thread starts workers, and the previous one has
  // finished (reconnect_in_progress_ was false), so join() returns at once.
  if (reconnect_thread_.joinable()) reconnect_thread_.join();
  reconnect_thread_ = std::thread(&PhidgetsBldcHardware::reconnect_worker, this, indices);
}

void PhidgetsBldcHardware::log_connection_summary()
{
  const int connected = std::count(attached_.begin(), attached_.end(), true);
  const int enabled = std::count(motor_enabled_.begin(), motor_enabled_.end(), true);
  const size_t total = joint_names_.size();

  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"),
              "Connection complete: %d/%zu motors attached, %d/%zu enabled",
              connected, total, enabled, total);
  if (connected < static_cast<int>(total)) {
    RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"), "Some motors failed to connect.");
  }
}

void PhidgetsBldcHardware::setup_ros_communication()
{
  telemetry_node_ = rclcpp::Node::make_shared("phidgets_bldc_hardware");

  motor_telemetry_pub_ =
    telemetry_node_->create_publisher<std_msgs::msg::String>(
      "/rover/drive/motor_telemetry", 10);


  kill_sub_ =
    telemetry_node_->create_subscription<std_msgs::msg::String>(
      "/rover/relay_board/command",
      10,
      [this](const std_msgs::msg::String::SharedPtr msg)
      {
        this->software_kill_callback(msg);
      });


  telemetry_executor_ =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  telemetry_executor_->add_node(telemetry_node_);


  telemetry_thread_ = std::thread([this]()
  {
    telemetry_executor_->spin();
  });


  RCLCPP_INFO(
    rclcpp::get_logger("PhidgetsBldcHardware"),
    "ROS communication started");
}


// ---------------------------------------------------------------------------
// Kill / power commands (telemetry thread)
//
// Command meanings, shared with jetson_relay and the dashboard:
//   {"type":"software_kill"}                     -> cut all motor power
//   {"type":"turn_on_all"}                       -> restore all motor power
//   {"type":"set_power","target":T,"enable":E}   -> E is the RELAY state:
//        enable=true  = relay energized = motor power CUT
//        enable=false = relay released  = motor power ON
//      T is "all" or a wheel joint name; other targets (lights) are ignored.
// ---------------------------------------------------------------------------

void PhidgetsBldcHardware::activate_software_kill(const std::string & reason)
{
  software_kill_active_ = true;
  wake_cv_.notify_all();   // abort any reconnect that is sleeping between retries

  RCLCPP_WARN(rclcpp::get_logger("PhidgetsBldcHardware"),
              "%s - Disabling all motors", reason.c_str());

  std::lock_guard<std::mutex> lk(hw_mutex_);
  for (size_t i = 0; i < joint_names_.size(); i++) {
    disable_motor(i, reason);
  }
}

void PhidgetsBldcHardware::software_kill_callback(const std_msgs::msg::String::SharedPtr msg)
{
  auto logger = rclcpp::get_logger("PhidgetsBldcHardware");
  nlohmann::json cmd;
  try {
    cmd = nlohmann::json::parse(msg->data);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "Failed to parse relay command: %s", e.what());
    return;
  }
  if (!cmd.is_object()) return;

  const std::string type = cmd.value("type", "");
  std::vector<size_t> all(joint_names_.size());
  std::iota(all.begin(), all.end(), 0);

  if (type == "software_kill") {
    activate_software_kill("SOFTWARE KILL");
  } else if (type == "turn_on_all") {
    software_kill_active_ = false;
    request_reconnect(all, "TURN ON ALL");
  } else if (type == "set_power") {
    if (!cmd.contains("enable") || !cmd["enable"].is_boolean()) {
      RCLCPP_WARN(logger, "set_power without boolean 'enable' ignored");
      return;
    }
    const bool relay_energized = cmd["enable"].get<bool>();
    const std::string target = cmd.value("target", "all");

    if (target == "all") {
      if (relay_energized) {
        activate_software_kill("SET_POWER all: relays ON (motor power cut)");
      } else {
        software_kill_active_ = false;
        request_reconnect(all, "SET_POWER all: relays OFF (motor power on)");
      }
      return;
    }

    const int i = find_joint(target);
    if (i < 0) return;   // not a drive motor (e.g. headlights)

    if (relay_energized) {
      std::lock_guard<std::mutex> lk(hw_mutex_);
      disable_motor(static_cast<size_t>(i), "Relay energized (power cut) for " + target);
    } else if (software_kill_active_) {
      RCLCPP_WARN(logger, "Not reconnecting %s: software kill is active (send turn_on_all)",
                  target.c_str());
    } else {
      request_reconnect({static_cast<size_t>(i)}, "SET_POWER " + target + " on");
    }
  }
}

void PhidgetsBldcHardware::publish_motor_telemetry()
{
  if (!motor_telemetry_pub_) return;
  
  rclcpp::Time now = telemetry_node_->now();
  if (last_telemetry_publish_time_.nanoseconds() != 0 &&
      (now - last_telemetry_publish_time_).seconds() < telemetry_publish_period_sec_) {
    return;
  }
  last_telemetry_publish_time_ = now;
  
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  stream << "{\"motors\":[";
  
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (i > 0) stream << ",";
    
    // Determine health status for dashboard
    std::string health;
    std::string fault;
    
    if (!motor_enabled_[i]) {
      health = "offline";
      fault = "Motor disabled";
    } else if (!attached_[i]) {
      health = "offline";
      fault = "Motor not attached";
    } else {
      health = (temperature_c_[i] < 70.0) ? "healthy" : "warning";
      fault = (temperature_c_[i] < 70.0) ? "None" : "High temperature";
    }
    
    stream << "{"
           << "\"name\":\"" << joint_names_[i] << "\","
           << "\"attached\":" << (attached_[i] ? "true" : "false") << ","
           << "\"position\":" << pos_rad_[i] << ","
           << "\"velocity\":" << vel_state_[i] << ","
           << "\"temperature\":" << temperature_c_[i] << ","
           << "\"health\":\"" << health << "\","
           << "\"fault\":\"" << fault << "\","
           << "\"enabled\":" << (motor_enabled_[i] ? "true" : "false") << ","
           << "\"last_update\":\"" << std::to_string(now.seconds()) << "\""
           << "}";
  }
  stream << "]}";
  
  std_msgs::msg::String msg;
  msg.data = stream.str();
  motor_telemetry_pub_->publish(msg);
}

// Hardware interface methods
std::vector<hardware_interface::StateInterface>
PhidgetsBldcHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    state_interfaces.emplace_back(joint_names_[i], "position", &pos_rad_[i]);
    state_interfaces.emplace_back(joint_names_[i], "velocity", &vel_state_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PhidgetsBldcHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    command_interfaces.emplace_back(joint_names_[i], "velocity", &cmd_[i]);
  }
  return command_interfaces;
}

void PhidgetsBldcHardware::disable_motor(size_t i, const std::string & reason)
{
  // Requires hw_mutex_.
  if (motors_[i]) {
    PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);  // best effort before close
    close_phidget(i);
  }
  if (motor_enabled_[i]) {
    RCLCPP_ERROR(rclcpp::get_logger("PhidgetsBldcHardware"),
                 "Motor %zu (%s) DISABLED - %s", i, joint_names_[i].c_str(), reason.c_str());
  }
  motor_enabled_[i] = false;
  attached_[i] = false;
  failsafe_state_[i] = FailsafeState::NOT_ENABLED;
  pos_rad_[i] = 0.0;
  vel_state_[i] = 0.0;
  cmd_[i] = 0.0;
}

void PhidgetsBldcHardware::service_failsafe(size_t i)
{
  // Requires hw_mutex_ and a valid motors_[i].
  if (failsafe_timeout_ms_ == 0) return;
  auto logger = rclcpp::get_logger("PhidgetsBldcHardware");
  const auto now = std::chrono::steady_clock::now();

  if (failsafe_state_[i] == FailsafeState::NOT_ENABLED) {
    uint32_t timeout_ms = failsafe_timeout_ms_;
    uint32_t min_ms = 0;
    if (PhidgetBLDCMotor_getMinFailsafeTime(motors_[i], &min_ms) == EPHIDGET_OK &&
        timeout_ms < min_ms)
    {
      RCLCPP_WARN(logger, "Motor %zu: failsafe %u ms below device minimum, using %u ms",
                  i, timeout_ms, min_ms);
      timeout_ms = min_ms;
    }
    if (PhidgetBLDCMotor_enableFailsafe(motors_[i], timeout_ms) == EPHIDGET_OK) {
      failsafe_state_[i] = FailsafeState::ENABLED;
      last_failsafe_reset_[i] = now;
      RCLCPP_INFO(logger, "Motor %zu (%s) failsafe ARMED (%u ms)",
                  i, joint_names_[i].c_str(), timeout_ms);
    } else {
      failsafe_state_[i] = FailsafeState::UNSUPPORTED;   // don't retry every cycle
      RCLCPP_ERROR(logger, "Motor %zu (%s): could not enable Phidget failsafe! "
                   "Motor will NOT stop on its own if this process freezes.",
                   i, joint_names_[i].c_str());
    }
    return;
  }

  if (failsafe_state_[i] != FailsafeState::ENABLED) return;

  // Reset 4x per timeout period: plenty of margin without flooding the VINT bus.
  const auto period = std::chrono::milliseconds(std::max<uint32_t>(failsafe_timeout_ms_ / 4, 20));
  if (now - last_failsafe_reset_[i] < period) return;

  if (PhidgetBLDCMotor_resetFailsafe(motors_[i]) != EPHIDGET_OK) {
    // Most likely the failsafe already tripped (e.g. a long stall of this
    // thread). The channel refuses commands until reopened, so latch it off;
    // the operator re-arms with turn_on_all.
    disable_motor(i, "Failsafe tripped / reset failed - press TURN ON to re-arm");
    return;
  }
  last_failsafe_reset_[i] = now;
}

PhidgetsBldcHardware::~PhidgetsBldcHardware()
{
  shutting_down_ = true;
  wake_cv_.notify_all();

  // Stop ROS thread (no more kill/power callbacks after this)
  if (telemetry_executor_) {
    telemetry_executor_->cancel();
  }
  if (telemetry_thread_.joinable()) {
    telemetry_thread_.join();
  }

  // Wait for a reconnect in progress (aborts at its next check, <= ~3.5 s)
  if (reconnect_thread_.joinable()) {
    reconnect_thread_.join();
  }

  std::lock_guard<std::mutex> lk(hw_mutex_);
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (motors_[i]) PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
    close_phidget(i);
    close_temperature_sensor(i);
  }
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Activated. Motors ready.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PhidgetsBldcHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("PhidgetsBldcHardware"), "Deactivating...");

  std::lock_guard<std::mutex> lk(hw_mutex_);
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (motors_[i]) PhidgetBLDCMotor_setTargetVelocity(motors_[i], 0.0);
    close_phidget(i);
    close_temperature_sensor(i);
    attached_[i] = false;
    temperature_attached_[i] = false;
    motor_enabled_[i] = false;
    failsafe_state_[i] = FailsafeState::NOT_ENABLED;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PhidgetsBldcHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lk(hw_mutex_);

  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (!motor_enabled_[i] || !attached_[i] || !motors_[i]) {
      vel_state_[i] = 0.0;
      pos_rad_[i] = 0.0;
      continue;
    }

    double vel_duty = 0.0, pos_val = 0.0;

    if (PhidgetBLDCMotor_getVelocity(motors_[i], &vel_duty) != EPHIDGET_OK) {
      disable_motor(i, "Velocity read failed during operation");
      continue;
    }

    if (PhidgetBLDCMotor_getPosition(motors_[i], &pos_val) != EPHIDGET_OK) {
      disable_motor(i, "Position read failed during operation");
      continue;
    }

    vel_state_[i] = vel_duty * direction_sign_[i];
    pos_rad_[i] = pos_val * 2.0 * M_PI * direction_sign_[i];

    if (temperature_sensors_[i]) {
      double temp = 0.0;
      if (PhidgetTemperatureSensor_getTemperature(temperature_sensors_[i], &temp) == EPHIDGET_OK) {
        temperature_c_[i] = temp;
      }
    }
  }

  publish_motor_telemetry();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PhidgetsBldcHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lk(hw_mutex_);

  // Killed motors are closed already; never command anything while killed.
  if (software_kill_active_) return hardware_interface::return_type::OK;

  // Calculate average command per side based on ENABLED motors only
  double left_sum = 0.0, right_sum = 0.0;
  int left_count = 0, right_count = 0;

  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (!motor_enabled_[i] || !attached_[i]) continue;

    if (is_left_wheel_[i]) {
      left_sum += cmd_[i];
      left_count++;
    } else {
      right_sum += cmd_[i];
      right_count++;
    }
  }

  double left_target = (left_count > 0) ? left_sum / left_count : 0.0;
  double right_target = (right_count > 0) ? right_sum / right_count : 0.0;
  const bool distribute_by_axle = both_middle_wheels_attached();

  // Send commands to enabled motors
  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (!motor_enabled_[i] || !attached_[i] || !motors_[i]) continue;

    const double target = distribute_by_axle ?
      axle_weighted_command(i, left_target, right_target) :
      (is_left_wheel_[i] ? left_target : right_target);
    const double duty = clamp(target * direction_sign_[i], -command_limit_, command_limit_);

    if (PhidgetBLDCMotor_setTargetVelocity(motors_[i], duty) != EPHIDGET_OK) {
      disable_motor(i, "Command send failed during operation");
      continue;
    }

    service_failsafe(i);
  }

  return hardware_interface::return_type::OK;
}

void PhidgetsBldcHardware::close_phidget(size_t i)
{
  if (motors_[i]) {
    Phidget_close((PhidgetHandle)motors_[i]);
    PhidgetBLDCMotor_delete(&motors_[i]);
    motors_[i] = nullptr;
  }
}

void PhidgetsBldcHardware::close_temperature_sensor(size_t i)
{
  if (temperature_sensors_[i]) {
    Phidget_close((PhidgetHandle)temperature_sensors_[i]);
    PhidgetTemperatureSensor_delete(&temperature_sensors_[i]);
    temperature_sensors_[i] = nullptr;
  }
}

}  // namespace phidgets_hardware

PLUGINLIB_EXPORT_CLASS(phidgets_hardware::PhidgetsBldcHardware, hardware_interface::SystemInterface)
