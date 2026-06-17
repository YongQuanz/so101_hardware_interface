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
  return hardware_interface::CallbackReturn::SUCCESS;
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
