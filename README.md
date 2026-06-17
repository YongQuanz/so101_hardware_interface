# so101_hardware_interface

A `ros2_control` **SystemInterface** plugin for the SO-101 robot arm,
using the **Waveshare Bus Servo Adapter (A)** with ST/STS-series bus servos.

## Hardware

| Component | Detail |
|---|---|
| Adapter board | [Waveshare Bus Servo Adapter (A)](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) |
| Interface | USB-C → UART half-duplex |
| Protocol | Feetech ST / STS serial bus (1-wire UART) |
| Default baud | 1 000 000 |
| Servo range | 0–4095 raw steps = 0–360° (12-bit) |
| Supported servos | ST3215, STS3215, and other ST-series models |

## Package structure

```
so101_hardware_interface/
├── include/so101_hardware_interface/
│   ├── so101_hardware_interface.hpp   # ros2_control plugin class
│   └── st_servo_driver.hpp            # Waveshare SDK wrapper
├── src/
│   ├── so101_hardware_interface.cpp
│   └── st_servo_driver.cpp
├── vendor/
│   └── SCServo/                       # ← clone SDK here (see below)
│       └── SCServo.h                  # placeholder / setup instructions
├── config/
│   ├── so101_ros2_control.urdf.xacro  # <ros2_control> hardware tag
│   └── so101_controllers.yaml
├── launch/
│   └── so101_bringup.launch.py
├── so101_hardware_interface_plugin.xml
├── CMakeLists.txt
└── package.xml
```

## 1 — Clone the SCServo SDK

The hardware driver wraps Waveshare's official C++ Linux SDK.
Clone it into `vendor/SCServo/` before building:

```bash
cd so101_hardware_interface/vendor/SCServo
git clone https://github.com/waveshare/SCServo_Linux.git .
```

> **Fallback:** if you downloaded the SDK zip from the Waveshare wiki, extract
> it so that `vendor/SCServo/CMakeLists.txt` exists.

Once the SDK is present, remove the `#error` line from `vendor/SCServo/SCServo.h`
(the placeholder) and replace the stub include in `src/st_servo_driver.cpp`:

```cpp
// Replace:
// #ifndef SCSERVO_SDK_FOUND  ... struct SMS_STS { ... };
// With:
#include <SCServo.h>
```

## 2 — Assign servo IDs

Each physical servo must have a unique bus ID (1–253).
Factory default is **1**. Use the SCServo `ChangeID` example to re-assign:

```bash
cd vendor/SCServo/examples/ChangeID
make
./ChangeID /dev/ttyUSB0 1 2   # change ID 1 → 2
```

Then update the `<param name="servo_id">` values in
`config/so101_ros2_control.urdf.xacro` to match.

## 3 — Build

```bash
cd ~/ros2_ws
sudo apt install ros-$ROS_DISTRO-ros2-control ros-$ROS_DISTRO-ros2-controllers
colcon build --packages-select so101_hardware_interface
source install/setup.bash
```

## 4 — Launch

```bash
# Default port /dev/ttyUSB0
ros2 launch so101_hardware_interface so101_bringup.launch.py

# Override port or motion parameters
ros2 launch so101_hardware_interface so101_bringup.launch.py \
  device_port:=/dev/ttyACM0 \
  use_rviz:=true
```

## 5 — Verify

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
ros2 topic echo /joint_states
```

## Unit conversions

The `StServoDriver` maps automatically between ROS (radians) and servo raw units:

```
raw = rad × (4096 / 2π) + 2048     # 2048 = centre / 0 rad
rad = (raw − 2048) × (2π / 4096)
```

Speed conversion: `1 raw_step/s ≈ 0.001534 rad/s`

## URDF parameters reference

| Parameter | Default | Description |
|---|---|---|
| `device_port` | `/dev/ttyUSB0` | Serial port of the Bus Servo Adapter |
| `baud_rate` | `1000000` | UART baud rate |
| `timeout` | `1.0` | Comm timeout (s) |
| `default_speed` | `0` | Motion speed in raw steps/s (0 = max) |
| `default_acc` | `50` | Acceleration 0–254 (0 = no limit) |
| `servo_id` *(per joint)* | — | **Required.** Physical servo bus ID |
