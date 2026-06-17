#ifndef SO101_HARDWARE_INTERFACE__SO101_HARDWARE_INTERFACE_HPP_
#define SO101_HARDWARE_INTERFACE__SO101_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "so101_hardware_interface/st_servo_driver.hpp"

namespace so101_hardware_interface
{

class So101HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(So101HardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware parameters (from URDF <param> tags)
  std::string          device_port_;
  int                  baud_rate_;
  double               timeout_;
  uint16_t             default_speed_;  // raw steps/s  (0 = max)
  uint8_t              default_acc_;    // 0–254        (0 = no limit)

  // Servo IDs parsed from URDF joint parameters
  std::vector<uint8_t> servo_ids_;

  // Joint state / command buffers (indexed parallel to info_.joints)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_positions_;

  // Waveshare ST-servo driver
  std::unique_ptr<StServoDriver> driver_;

  rclcpp::Logger logger_{rclcpp::get_logger("So101HardwareInterface")};
};

}  // namespace so101_hardware_interface

#endif  // SO101_HARDWARE_INTERFACE__SO101_HARDWARE_INTERFACE_HPP_
