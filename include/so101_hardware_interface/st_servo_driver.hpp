#ifndef SO101_HARDWARE_INTERFACE__ST_SERVO_DRIVER_HPP_
#define SO101_HARDWARE_INTERFACE__ST_SERVO_DRIVER_HPP_

/**
 * @file st_servo_driver.hpp
 * @brief Thin C++ wrapper around the Waveshare / Feetech SCServo Linux SDK.
 *
 * Responsibilities:
 *  - Open / close the UART port (via SMS_STS::begin / end)
 *  - Convert between ROS units (radians, rad/s) and servo raw units (steps)
 *  - Read position + velocity for a set of servo IDs
 *  - Write position commands (single and synchronised)
 *  - Enable / disable torque per servo
 *
 * Coordinate convention
 * ─────────────────────
 * ST3215 / STS3215 servos:
 *   raw range  : 0 … 4095  (12-bit)
 *   full range : 0° … 360°  (or ≈ -π … π if you set the zero at 2048)
 *
 * This driver uses:
 *   radians = (raw - zero_offset_) * RAW_TO_RAD
 *   raw     = radians * RAD_TO_RAW + zero_offset_
 *
 * where RAW_TO_RAD = 2π / 4096  ≈ 0.001534 rad/step
 *       RAD_TO_RAW = 4096 / 2π  ≈ 651.9 steps/rad
 */

#include <cstdint>
#include <string>
#include <vector>

// ── SCServo SDK forward declaration ──────────────────────────────────────────
// Include the real header once you have cloned the SDK (see vendor/SCServo/).
// For now we forward-declare SMS_STS so the wrapper header compiles cleanly.
class SMS_STS;  // defined in vendor/SCServo/SMS_STS.h (part of SCServo SDK)

namespace so101_hardware_interface
{

// Unit conversion constants for ST-series servos (12-bit, 0–4095 = 0–360°)
static constexpr double RAW_TO_RAD = 2.0 * M_PI / 4096.0;  // ~0.001534 rad/step
static constexpr double RAD_TO_RAW = 4096.0 / (2.0 * M_PI); // ~651.9 steps/rad
static constexpr int16_t SERVO_CENTER = 2048;                 // 180° / zero point

struct ServoState
{
  double position_rad = 0.0;
  double velocity_rad_s = 0.0;
  bool   comm_ok = false;
};

class StServoDriver
{
public:
  explicit StServoDriver(
    const std::string & port,
    int baud_rate,
    const std::vector<uint8_t> & servo_ids);

  ~StServoDriver();

  /// Open the serial port and verify communication with all configured servos.
  bool open();

  /// Close the serial port gracefully.
  void close();

  /// Enable or disable torque on a single servo.
  bool set_torque(uint8_t id, bool enable);

  /// Enable / disable torque on all configured servos.
  bool set_torque_all(bool enable);

  /**
   * Read position (rad) and velocity (rad/s) for all configured servos.
   * @param[out] states  Vector sized to match servo_ids_.
   * @return true if ALL reads succeeded.
   */
  bool read_all(std::vector<ServoState> & states);

  /**
   * Write a position command (rad) to a single servo.
   * @param id       Servo hardware ID (1–253)
   * @param pos_rad  Target position in radians (centred at 0 = middle)
   * @param speed    Motion speed in raw steps/s (0 = max)
   * @param acc      Acceleration (0–254; 0 = no limit)
   */
  bool write_position(uint8_t id, double pos_rad, uint16_t speed = 0, uint8_t acc = 0);

  /**
   * Synchronised write to all servos in one bus transaction.
   * @param pos_rad  Positions in radians, indexed parallel to servo_ids_.
   * @param speed    Common speed for all servos (raw steps/s; 0 = max)
   * @param acc      Common acceleration (0 = no limit)
   */
  bool sync_write_positions(
    const std::vector<double> & pos_rad,
    uint16_t speed = 0,
    uint8_t acc = 0);

  const std::vector<uint8_t> & servo_ids() const { return servo_ids_; }

private:
  int16_t rad_to_raw(double rad) const;
  double  raw_to_rad(int16_t raw) const;

  std::string          port_;
  int                  baud_rate_;
  std::vector<uint8_t> servo_ids_;
  SMS_STS *            sms_sts_{nullptr};   ///< Owning pointer; created in open()
};

}  // namespace so101_hardware_interface

#endif  // SO101_HARDWARE_INTERFACE__ST_SERVO_DRIVER_HPP_
