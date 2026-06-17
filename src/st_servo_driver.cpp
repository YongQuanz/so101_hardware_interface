#include "so101_hardware_interface/st_servo_driver.hpp"
#include <cmath>
#include <stdexcept>
#include <SCServo.h>
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware_interface
{

namespace
{
// Raw speed from ReadSpeed() is signed (negative = CCW).
// Convert to rad/s: 1 step/s ≈ RAW_TO_RAD rad/s
double raw_speed_to_rad_s(int raw_speed)
{
  return static_cast<double>(raw_speed) * RAW_TO_RAD;
}
}  // namespace

// ── Constructor / Destructor ──────────────────────────────────────────────────

StServoDriver::StServoDriver(
  const std::string & port,
  int baud_rate,
  const std::vector<uint8_t> & servo_ids)
: port_(port), baud_rate_(baud_rate), servo_ids_(servo_ids)
{}

StServoDriver::~StServoDriver()
{
  close();
  delete sms_sts_;
}

// ── Open / Close ──────────────────────────────────────────────────────────────

bool StServoDriver::open()
{
  if (sms_sts_) {
    // Already open — close first.
    sms_sts_->end();
    delete sms_sts_;
    sms_sts_ = nullptr;
  }

  sms_sts_ = new SMS_STS();

  if (!sms_sts_->begin(baud_rate_, port_.c_str())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("StServoDriver"),
      "Failed to open serial port '%s' at %d baud.", port_.c_str(), baud_rate_);
    delete sms_sts_;
    sms_sts_ = nullptr;
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("StServoDriver"),
    "Opened '%s' at %d baud. Pinging %zu servo(s)...",
    port_.c_str(), baud_rate_, servo_ids_.size());

  // Verify each servo responds.
  bool all_ok = true;
  for (uint8_t id : servo_ids_) {
    int pos = sms_sts_->ReadPos(id);
    if (pos < 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("StServoDriver"),
        "No response from servo ID %u on '%s'. Check ID, power, and wiring.",
        id, port_.c_str());
      all_ok = false;
    } else {
      RCLCPP_INFO(
        rclcpp::get_logger("StServoDriver"),
        "  Servo ID %u → pos=%d (%.3f rad) ✓", id, pos, raw_to_rad(static_cast<int16_t>(pos)));
    }
  }

  return all_ok;
}

void StServoDriver::close()
{
  if (sms_sts_) {
    sms_sts_->end();
  }
}

// ── Torque ────────────────────────────────────────────────────────────────────

bool StServoDriver::set_torque(uint8_t id, bool enable)
{
  if (!sms_sts_) { return false; }
  int ret = sms_sts_->EnableTorque(id, enable ? 1 : 0);
  if (ret < 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("StServoDriver"),
      "EnableTorque(id=%u, %s) failed (ret=%d).", id, enable ? "ON" : "OFF", ret);
    return false;
  }
  return true;
}

bool StServoDriver::set_torque_all(bool enable)
{
  bool ok = true;
  for (uint8_t id : servo_ids_) {
    ok &= set_torque(id, enable);
  }
  return ok;
}

// ── Read ──────────────────────────────────────────────────────────────────────

bool StServoDriver::read_all(std::vector<ServoState> & states)
{
  if (!sms_sts_) { return false; }

  states.resize(servo_ids_.size());
  bool all_ok = true;

  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    uint8_t id = servo_ids_[i];

    int raw_pos   = sms_sts_->ReadPos(id);
    int raw_speed = sms_sts_->ReadSpeed(id);

    if (raw_pos < 0 || raw_speed < 0) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("StServoDriver"),
        *rclcpp::Clock::make_shared(),
        2000 /* ms */,
        "Read failed for servo ID %u (pos=%d, speed=%d).", id, raw_pos, raw_speed);
      states[i].comm_ok = false;
      all_ok = false;
    } else {
      states[i].position_rad  = raw_to_rad(static_cast<int16_t>(raw_pos));
      states[i].velocity_rad_s = raw_speed_to_rad_s(raw_speed);
      states[i].comm_ok = true;
    }
  }

  return all_ok;
}

// ── Write ─────────────────────────────────────────────────────────────────────

bool StServoDriver::write_position(
  uint8_t id, double pos_rad, uint16_t speed, uint8_t acc)
{
  if (!sms_sts_) { return false; }

  int16_t raw = rad_to_raw(pos_rad);
  int ret = sms_sts_->WritePosEx(id, raw, speed, acc);
  if (ret < 0) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("StServoDriver"),
      *rclcpp::Clock::make_shared(),
      2000,
      "WritePosEx failed for servo ID %u (raw=%d, ret=%d).", id, raw, ret);
    return false;
  }
  return true;
}

bool StServoDriver::sync_write_positions(
  const std::vector<double> & pos_rad,
  uint16_t speed,
  uint8_t acc)
{
  if (!sms_sts_) { return false; }
  if (pos_rad.size() != servo_ids_.size()) { return false; }

  const size_t n = servo_ids_.size();

  // SyncWritePosEx expects plain C arrays.
  std::vector<uint8_t>  ids(n);
  std::vector<int16_t>  positions(n);
  std::vector<uint16_t> speeds(n, speed);
  std::vector<uint8_t>  accs(n, acc);

  for (size_t i = 0; i < n; ++i) {
    ids[i]       = servo_ids_[i];
    positions[i] = rad_to_raw(pos_rad[i]);
  }

  int ret = sms_sts_->SyncWritePosEx(
    ids.data(),
    static_cast<uint8_t>(n),
    positions.data(),
    speeds.data(),
    accs.data());

  if (ret < 0) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("StServoDriver"),
      *rclcpp::Clock::make_shared(),
      2000,
      "SyncWritePosEx failed (ret=%d).", ret);
    return false;
  }
  return true;
}

// ── Unit helpers ──────────────────────────────────────────────────────────────

int16_t StServoDriver::rad_to_raw(double rad) const
{
  // Centre raw range at SERVO_CENTER (2048 = 0 rad)
  double raw_d = rad * RAD_TO_RAW + static_cast<double>(SERVO_CENTER);
  // Clamp to valid servo range [0, 4095]
  raw_d = std::max(0.0, std::min(4095.0, raw_d));
  return static_cast<int16_t>(std::lround(raw_d));
}

double StServoDriver::raw_to_rad(int16_t raw) const
{
  return (static_cast<double>(raw) - static_cast<double>(SERVO_CENTER)) * RAW_TO_RAD;
}

}  // namespace so101_hardware_interface
