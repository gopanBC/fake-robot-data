# Fake Diffdrive ROS 2 Node

This package provides a **ROS 2 node** that simulates a differential-drive robot by publishing:

- `cmd_vel` (geometry_msgs/Twist)
- `odom` (nav_msgs/Odometry)
- `imu` (sensor_msgs/Imu)

The node supports configurable motion patterns including:

- **Straight line motion** (given distance & linear speed)
- **In-place rotation** (given angle & angular speed)
- **Curved motion** (constant linear & angular velocities)

This is useful for testing localization, navigation, and robot controllers without real hardware.

---

## Installation

```bash
cd ~/ros2_ws/src
git clone <your_repo_url> fake_diffdrive
cd ~/ros2_ws
colcon build --packages-select fake_diffdrive
source install/setup.bash
```

---

## Configuration

You can configure the motion parameters via **config/config.yaml**:

```yaml
fake_diffdrive_node:
  ros__parameters:
    motion_type: "straight"    # straight | turn | curve
    linear_speed: 0.2          # m/s
    angular_speed: 0.0         # rad/s
    target_distance: 1.0       # m
    target_angle: 1.57         # rad
```

Alternatively, you can override parameters from the command line:

```bash
ros2 run fake_diffdrive fake_diffdrive_node --ros-args -p motion_type:=curve -p linear_speed:=0.3 -p angular_speed:=0.5
```

---

## Launch

Use the provided launch file:

```bash
ros2 launch fake_diffdrive fake_diffdrive.launch.py
```

This will start the node with parameters loaded from `config/config.yaml`.

---

## Topics

| Topic       | Type                         | Description |
|-------------|-----------------------------|-------------|
| `/cmd_vel`  | `geometry_msgs/msg/Twist`    | Commanded velocities being simulated |
| `/odom`     | `nav_msgs/msg/Odometry`      | Fake odometry computed from simulated motion |
| `/imu`      | `sensor_msgs/msg/Imu`        | Fake IMU orientation and angular velocity |

---

## Parameters

| Parameter         | Type    | Default | Description |
|------------------|--------|---------|-------------|
| `motion_type`     | string | straight | Type of motion: straight, turn, or curve |
| `linear_speed`    | double | 0.2     | Linear velocity (m/s) |
| `angular_speed`   | double | 0.0     | Angular velocity (rad/s) |
| `target_distance` | double | 1.0     | Distance to travel in straight mode (m) |
| `target_angle`    | double | 1.57    | Angle to rotate in turn mode (rad) |

---

## Example Use Cases

- Testing **localization** nodes (AMCL, SLAM) without a physical robot.
- Simulating motion for **path planning** validation.
- Creating reproducible scenarios for **control tuning**.

---

## Future Improvements

- Add ROS 2 service to dynamically switch motion profiles.
- Add noise models to odometry & IMU for realism.
- Add tf broadcaster for odom → base_link transform.

---

## License

This package is licensed under the **Apache 2.0 License**.
