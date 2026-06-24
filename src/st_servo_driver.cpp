#include "so101_hardware_interface/st_servo_driver.hpp"
#include <cmath>
#include <stdexcept>
#include <SCServo.h>
#include "rclcpp/rclcpp.hpp"

namespace so101_hardware_interface
{

namespace
{
// Shared steady clock used only for RCLCPP_WARN_THROTTLE calls.
// Constructed once at first use; avoids creating a new Clock on every read/write.
rclcpp::Clock & throttle_clock()
{
  static rclcpp::Clock clk(RCL_STEADY_TIME);
  return clk;
}

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
}

// ── Open / Close ──────────────────────────────────────────────────────────────

bool StServoDriver::open()
{
  if (sms_sts_) {
    // Already open — close first.
    sms_sts_->end();
    sms_sts_.reset();
  }

  sms_sts_ = std::make_unique<SMS_STS>();

  if (!sms_sts_->begin(baud_rate_, port_.c_str())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("StServoDriver"),
      "Failed to open serial port '%s' at %d baud.", port_.c_str(), baud_rate_);
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
//
// FeedBack(id) does ONE serial round-trip that bulk-reads all state registers
// into sms_sts_->Mem[].  The subsequent ReadPos(-1) / ReadSpeed(-1) calls pull
// from that local cache — zero serial traffic.
//
// Old approach: 2 round-trips × 6 servos = 12 transactions (~42 ms)
// New approach: 1 round-trip × 6 servos =  6 transactions (~7 ms)

bool StServoDriver::read_all(std::vector<ServoState> & states)
{
  if (!sms_sts_) { return false; }

  states.resize(servo_ids_.size());
  bool all_ok = true;

  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    uint8_t id = servo_ids_[i];

    // Single bulk read — populates sms_sts_->Mem[]
    int nlen = sms_sts_->FeedBack(id);

    if (nlen < 0) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("StServoDriver"),
        throttle_clock(),
        2000 /* ms */,
        "FeedBack failed for servo ID %u (ret=%d).", id, nlen);
      states[i].comm_ok = false;
      all_ok = false;
      continue;
    }

    // Pull from cache — no serial I/O
    int raw_pos   = sms_sts_->ReadPos(-1);
    int raw_speed = sms_sts_->ReadSpeed(-1);

    if (raw_pos < 0) {
    // raw_pos < 0 is the genuine error return from ReadPos(-1)
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("StServoDriver"),
      throttle_clock(),
      2000,
      "Bad cached position for servo ID %u (pos=%d).", id, raw_pos);
    states[i].comm_ok = false;
    all_ok = false;
    } else {
      // negative speed is valid — CCW motion
      states[i].position_rad   = raw_to_rad(static_cast<int16_t>(raw_pos));
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
      throttle_clock(),
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
  // Delegate to per-joint overload with uniform values
  const size_t n = servo_ids_.size();
  return sync_write_positions(
    pos_rad,
    std::vector<uint16_t>(n, speed),
    std::vector<uint8_t> (n, acc));
}

bool StServoDriver::sync_write_positions(
  const std::vector<double>   & pos_rad,
  const std::vector<uint16_t> & speeds,
  const std::vector<uint8_t>  & accs)
{
  if (!sms_sts_) { return false; }

  const size_t n = servo_ids_.size();

  if (pos_rad.size() != n || speeds.size() != n || accs.size() != n) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("StServoDriver"),
      throttle_clock(),
      2000,
      "sync_write_positions: size mismatch (ids=%zu pos=%zu speeds=%zu accs=%zu)",
      n, pos_rad.size(), speeds.size(), accs.size());
    return false;
  }

  // SyncWritePosEx expects plain C arrays
  std::vector<uint8_t>  ids(n);
  std::vector<int16_t>  positions(n);
  std::vector<uint16_t> spd(speeds.begin(), speeds.end());
  std::vector<uint8_t>  acc(accs.begin(),   accs.end());

  for (size_t i = 0; i < n; ++i) {
    ids[i]       = servo_ids_[i];
    positions[i] = rad_to_raw(pos_rad[i]);
  }

  sms_sts_->SyncWritePosEx(
    ids.data(),
    static_cast<uint8_t>(n),
    positions.data(),
    spd.data(),
    acc.data());

  // SyncWritePosEx is broadcast — no per-servo ACK on the bus.
  // Correctness is verified on the next read_all() cycle via FeedBack().
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