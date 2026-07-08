#include "so101_hardware_interface/so101_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware_interface
{

// ── on_init ───────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn So101HardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ── Hardware-level parameters ────────────────────────────────────────────
  auto param = [&](const std::string & key, const std::string & fallback) {
    return info_.hardware_parameters.count(key)
      ? info_.hardware_parameters.at(key)
      : fallback;
  };

  device_port_   = param("device_port",   "/dev/ttyUSB0");
  baud_rate_     = std::stoi(param("baud_rate",     "1000000"));
  timeout_       = std::stod(param("timeout",       "1.0"));
  default_speed_ = static_cast<uint16_t>(std::stoi(param("default_speed", "0")));
  default_acc_   = static_cast<uint8_t> (std::stoi(param("default_acc",   "0")));

  RCLCPP_INFO(logger_,
    "Port: %s  Baud: %d  Speed: %u  Acc: %u",
    device_port_.c_str(), baud_rate_, default_speed_, default_acc_);

  // ── Resize state / command buffers ───────────────────────────────────────
  const size_t n = info_.joints.size();
  hw_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.assign(n, std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  servo_ids_.resize(n);
  joint_accs_.resize(n);
  joint_speeds_.resize(n);

  // ── Validate each joint and extract servo ID ─────────────────────────────
  for (size_t i = 0; i < n; ++i) {
    const auto & joint = info_.joints[i];

    // Every joint must expose exactly one command interface: position
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(logger_,
        "Joint '%s' must have exactly 1 command interface (position). Found %zu.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Every joint must expose position + velocity state interfaces
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(logger_,
        "Joint '%s' must have 2 state interfaces (position, velocity). Found %zu.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // The URDF joint must declare a "servo_id" parameter
    if (!joint.parameters.count("servo_id")) {
      RCLCPP_FATAL(logger_,
        "Joint '%s' is missing required parameter 'servo_id' in the URDF <joint> block.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    servo_ids_[i] = static_cast<uint8_t>(std::stoi(joint.parameters.at("servo_id")));
    RCLCPP_INFO(logger_,
      "  Joint[%zu] '%s'  →  servo ID %u", i, joint.name.c_str(), servo_ids_[i]);

    // per-joint acc: use joint param if present, else fall back to default_acc_
    joint_accs_[i] = joint.parameters.count("acc")
      ? static_cast<uint8_t>(std::stoi(joint.parameters.at("acc")))
      : default_acc_;

    // per-joint speed: use joint param if present, else fall back to default_speed_
    joint_speeds_[i] = joint.parameters.count("speed")
      ? static_cast<uint16_t>(std::stoi(joint.parameters.at("speed")))
      : default_speed_;
  }

  RCLCPP_INFO(logger_, "on_init OK (%zu joints).", n);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_configure ──────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn So101HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Configuring – opening serial port...");

  // Zero all buffers
  std::fill(hw_positions_.begin(),          hw_positions_.end(),          0.0);
  std::fill(hw_velocities_.begin(),         hw_velocities_.end(),         0.0);
  std::fill(hw_commands_positions_.begin(), hw_commands_positions_.end(), 0.0);

  // Construct the driver (does NOT open port yet)
  driver_ = std::make_unique<StServoDriver>(device_port_, baud_rate_, servo_ids_);

  // Open port + ping all servos
  if (!driver_->open()) {
    RCLCPP_ERROR(logger_,
      "Failed to open/ping servos on '%s'. Check USB cable, power, and servo IDs.",
      device_port_.c_str());
    driver_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Standalone node + publisher for /servo_telemetry. We don't spin this
  // node — publishing doesn't require it, and read()/write() already run
  // on the controller_manager's own real-time-ish cycle.
  telemetry_node_ = std::make_shared<rclcpp::Node>("so101_hardware_interface_telemetry");
  telemetry_pub_  = telemetry_node_->create_publisher<so101_msgs::msg::ServoTelemetryArray>(
    "/servo_telemetry", rclcpp::SensorDataQoS());

  RCLCPP_INFO(logger_, "on_configure OK.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_activate ───────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn So101HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating – enabling torque...");

  if (!driver_ || !driver_->set_torque_all(true)) {
    RCLCPP_ERROR(logger_, "Failed to enable torque on all servos.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Sync command positions with current hardware positions to prevent jumps
  std::vector<ServoState> states;
  if (driver_->read_all(states)) {
    for (size_t i = 0; i < states.size(); ++i) {
      hw_positions_[i]           = states[i].position_rad;
      hw_velocities_[i]          = states[i].velocity_rad_s;
      hw_commands_positions_[i]  = states[i].position_rad;  // hold current
    }
  }

  RCLCPP_INFO(logger_, "on_activate OK.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_deactivate ─────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn So101HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating – disabling torque...");
  if (driver_) { driver_->set_torque_all(false); }
  RCLCPP_INFO(logger_, "on_deactivate OK.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_cleanup ────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn So101HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Cleaning up – closing serial port...");
  if (driver_) {
    driver_->close();
    driver_.reset();
  }
  teardown_telemetry_publisher();
  RCLCPP_INFO(logger_, "on_cleanup OK.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_shutdown ───────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn So101HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Shutting down.");
  if (driver_) {
    driver_->set_torque_all(false);
    driver_->close();
    driver_.reset();
  }
  teardown_telemetry_publisher();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── on_error ──────────────────────────────────────────────────────────────────

hardware_interface::CallbackReturn So101HardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(logger_, "Hardware error! Attempting safe shutdown.");
  if (driver_) {
    driver_->set_torque_all(false);
    driver_->close();
    driver_.reset();
  }
  teardown_telemetry_publisher();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ── teardown_telemetry_publisher ─────────────────────────────────────────────

void So101HardwareInterface::teardown_telemetry_publisher()
{
  telemetry_pub_.reset();
  telemetry_node_.reset();
}

// ── export_state_interfaces ───────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface>
So101HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

// ── export_command_interfaces ─────────────────────────────────────────────────

std::vector<hardware_interface::CommandInterface>
So101HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]);
  }
  return command_interfaces;
}

// ── read ──────────────────────────────────────────────────────────────────────

hardware_interface::return_type So101HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!driver_) { return hardware_interface::return_type::ERROR; }

  std::vector<ServoState> states;
  bool ok = driver_->read_all(states);

  for (size_t i = 0; i < states.size(); ++i) {
    if (states[i].comm_ok) {
      hw_positions_[i]  = states[i].position_rad;
      hw_velocities_[i] = states[i].velocity_rad_s;
    }
    // On comm failure: keep last known values – the controller will time out
    // if the issue persists beyond the trajectory's goal_time constraint.
  }

  // Publish full STS3215 telemetry — independent of comm_ok per-servo, so a
  // single dropped servo doesn't blank out the whole panel; its last-known
  // ServoState values just get republished until the next good read.
  if (telemetry_pub_ && telemetry_pub_->get_subscription_count() > 0) {
    so101_msgs::msg::ServoTelemetryArray msg;
    msg.header.stamp = telemetry_node_->now();
    msg.servos.reserve(states.size());

    for (size_t i = 0; i < states.size(); ++i) {
      so101_msgs::msg::ServoTelemetry t;
      t.id                  = servo_ids_[i];
      t.position_deg        = states[i].position_rad * 180.0 / M_PI;
      t.speed_steps_per_sec  = states[i].velocity_rad_s / RAW_TO_RAD;
      t.load_percent        = states[i].load_percent;
      t.voltage             = states[i].voltage;
      t.current_ma          = states[i].current_ma;
      t.temperature_c       = states[i].temperature_c;
      t.moving              = states[i].moving;
      t.error_voltage       = states[i].error_voltage;
      t.error_sensor        = states[i].error_sensor;
      t.error_temperature   = states[i].error_temperature;
      t.error_current       = states[i].error_current;
      t.error_angle         = states[i].error_angle;
      t.error_overload      = states[i].error_overload;
      msg.servos.push_back(t);
    }

    telemetry_pub_->publish(msg);
  }

  return ok ? hardware_interface::return_type::OK
            : hardware_interface::return_type::OK;  // degrade gracefully; don't crash
}

// ── write ─────────────────────────────────────────────────────────────────────

hardware_interface::return_type So101HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!driver_) { return hardware_interface::return_type::ERROR; }

  // Use SyncWritePosEx for a single bus transaction covering all joints.
  bool ok = driver_->sync_write_positions(
    hw_commands_positions_, default_speed_, default_acc_);

  return ok ? hardware_interface::return_type::OK
            : hardware_interface::return_type::OK;  // warn but don't crash
}

}  // namespace so101_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  so101_hardware_interface::So101HardwareInterface,
  hardware_interface::SystemInterface)