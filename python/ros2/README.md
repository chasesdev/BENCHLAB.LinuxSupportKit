# BenchLab ROS2 Integration

**⚠️ Status: Beta - Untested with Hardware**

Complete ROS2 implementation (1,692 lines of code) with production-grade architecture. **Requires hardware validation before production use.**

- ✅ No stubs or mocks - all features fully implemented
- ⚠️ Never tested with real BenchLab hardware
- ⚠️ Performance claims are estimates, not measurements
- ⚠️ 3 bugs fixed (AttributeError in service handlers)

See [`VALIDATION_STATUS.md`](./VALIDATION_STATUS.md) for detailed assessment and required work.

## Overview

This ROS2 integration provides two operational modes:

1. **Direct Serial Mode** (`benchlab_serial_node`) - Low-latency direct USB access (<10ms)
2. **HTTP Bridge Mode** (`benchlab_http_node`) - Uses existing bench labd service

Both modes provide:
- ✅ Structured ROS2 messages (type-safe, introspectable)
- ✅ Service servers for all 15 protocol commands
- ✅ Diagnostics publishing (`/diagnostics` topic)
- ✅ Lifecycle node management (configure/activate/deactivate)
- ✅ Configurable QoS policies (BEST_EFFORT for telemetry, RELIABLE for status)
- ✅ Multi-device support via namespaces
- ✅ Full protocol feature parity (fan control, RGB, calibration, actions)

## Package Structure

```
python/ros2/
├── benchlab_msgs/           # Message and service definitions
│   ├── msg/
│   │   ├── PowerSensor.msg
│   │   ├── FanSensor.msg
│   │   ├── BenchLabTelemetry.msg
│   │   ├── DeviceInfo.msg
│   │   ├── FanProfile.msg
│   │   ├── RGBConfig.msg
│   │   └── CalibrationData.msg
│   ├── srv/
│   │   ├── SetDeviceName.srv
│   │   ├── SetFanProfile.srv
│   │   ├── SetRGB.srv
│   │   ├── SetCalibration.srv
│   │   ├── LoadCalibration.srv
│   │   ├── StoreCalibration.srv
│   │   └── ExecuteAction.srv
│   ├── CMakeLists.txt
│   └── package.xml
│
└── benchlab_ros2/           # ROS2 nodes
    ├── benchlab_ros2/
    │   ├── __init__.py
    │   ├── binary_protocol.py       # Python implementation of binary protocol
    │   ├── benchlab_serial_node.py  # Direct serial node
    │   └── benchlab_http_node.py    # HTTP bridge node
    ├── launch/
    │   ├── benchlab_serial.launch.py
    │   ├── benchlab_http.launch.py
    │   └── benchlab_multi_device.launch.py
    ├── config/
    │   └── benchlab_params.yaml
    ├── test/
    ├── setup.py
    ├── setup.cfg
    └── package.xml
```

## Installation

### Prerequisites

- ROS2 (Humble, Iron, or Jazzy recommended)
- Python 3.8+
- For serial mode: pyserial (`pip install pyserial`)
- For HTTP mode: requests (`pip install requests`)

### Build and Install

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Clone or symlink the BenchLab packages
ln -s /path/to/BENCHLAB.LinuxSupportKit/python/ros2/benchlab_msgs .
ln -s /path/to/BENCHLAB.LinuxSupportKit/python/ros2/benchlab_ros2 .

# Install Python dependencies
pip install pyserial requests

# Build packages
cd ~/ros2_ws
colcon build --packages-select benchlab_msgs benchlab_ros2

# Source the workspace
source install/setup.bash
```

### Verify Installation

```bash
# Check packages are available
ros2 pkg list | grep benchlab

# Should show:
# benchlab_msgs
# benchlab_ros2

# List message types
ros2 interface list | grep benchlab

# List available nodes
ros2 run benchlab_ros2 <TAB>
```

## Quick Start

### Direct Serial Mode (Recommended for Robotics)

```bash
# Launch single device
ros2 launch benchlab_ros2 benchlab_serial.launch.py device_path:=/dev/benchlab0

# In another terminal, configure and activate (lifecycle node)
ros2 lifecycle set /benchlab_serial_node configure
ros2 lifecycle set /benchlab_serial_node activate

# View telemetry
ros2 topic echo /benchlab/telemetry
```

### HTTP Bridge Mode

```bash
# Ensure benchlabd service is running first
# (see main README for benchlabd setup)

# Launch HTTP bridge
ros2 launch benchlab_ros2 benchlab_http.launch.py device_id:=benchlab0

# Configure and activate
ros2 lifecycle set /benchlab_http_node configure
ros2 lifecycle set /benchlab_http_node activate

# View telemetry
ros2 topic echo /benchlab/telemetry
```

## Topics

### Published Topics

| Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/benchlab/telemetry` | `BenchLabTelemetry` | BEST_EFFORT, depth=10 | Sensor telemetry (10Hz default) |
| `/benchlab/device_info` | `DeviceInfo` | RELIABLE, TRANSIENT_LOCAL, depth=1 | Device information |
| `/diagnostics` | `DiagnosticArray` | RELIABLE, depth=10 | Node diagnostics |

### Subscribing to Telemetry

```python
import rclpy
from rclpy.node import Node
from benchlab_msgs.msg import BenchLabTelemetry

class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('telemetry_subscriber')
        self.subscription = self.create_subscription(
            BenchLabTelemetry,
            '/benchlab/telemetry',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(f'Chip temp: {msg.chip_temp_c}°C')
        self.get_logger().info(f'Total power: {sum(p.power_w for p in msg.power):.2f}W')
        self.get_logger().info(f'Fan 0: {msg.fans[0].rpm} RPM')

def main():
    rclpy.init()
    node = TelemetrySubscriber()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

## Services

All service servers are available for bidirectional control:

| Service | Type | Description |
|---------|------|-------------|
| `/benchlab/set_name` | `SetDeviceName` | Set device name |
| `/benchlab/set_fan_profile` | `SetFanProfile` | Configure fan (0-8) |
| `/benchlab/set_rgb` | `SetRGB` | Configure RGB LEDs |
| `/benchlab/set_calibration` | `SetCalibration` | Apply calibration to RAM |
| `/benchlab/load_calibration` | `LoadCalibration` | Load from flash |
| `/benchlab/store_calibration` | `StoreCalibration` | Save to flash |
| `/benchlab/execute_action` | `ExecuteAction` | Execute action (0-255) |

### Calling Services

#### Set Fan Profile (Manual Mode)

```bash
ros2 service call /benchlab/set_fan_profile benchlab_msgs/srv/SetFanProfile "
fan_index: 0
profile:
  mode: 0
  manual_duty: 128
  temp_threshold_c: 0.0
  min_duty: 0
  max_duty: 255
  sensor_index: 0
"
```

#### Set Fan Profile (Auto Mode)

```bash
ros2 service call /benchlab/set_fan_profile benchlab_msgs/srv/SetFanProfile "
fan_index: 0
profile:
  mode: 1
  manual_duty: 0
  temp_threshold_c: 60.0
  min_duty: 64
  max_duty: 255
  sensor_index: 0
"
```

#### Set RGB LED (Solid Color)

```bash
ros2 service call /benchlab/set_rgb benchlab_msgs/srv/SetRGB "
config:
  mode: 1
  red: 255
  green: 0
  blue: 0
  brightness: 128
  speed: 0
"
```

#### Set Device Name

```bash
ros2 service call /benchlab/set_name benchlab_msgs/srv/SetDeviceName "
name: 'TestRig01'
"
```

### Service Call from Python

```python
import rclpy
from rclpy.node import Node
from benchlab_msgs.srv import SetFanProfile
from benchlab_msgs.msg import FanProfile

class FanController(Node):
    def __init__(self):
        super().__init__('fan_controller')
        self.client = self.create_client(SetFanProfile, '/benchlab/set_fan_profile')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def set_fan_auto(self, fan_index, temp_threshold, min_duty, max_duty):
        request = SetFanProfile.Request()
        request.fan_index = fan_index
        request.profile = FanProfile()
        request.profile.mode = 1  # Auto
        request.profile.temp_threshold_c = temp_threshold
        request.profile.min_duty = min_duty
        request.profile.max_duty = max_duty
        request.profile.sensor_index = 0

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        self.get_logger().info(f'Fan set: {response.message}')

def main():
    rclpy.init()
    controller = FanController()
    controller.set_fan_auto(fan_index=0, temp_threshold=60.0, min_duty=64, max_duty=255)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Multi-Device Support

Use namespaces to run multiple devices simultaneously:

```bash
# Launch multi-device (edit launch file for your devices)
ros2 launch benchlab_ros2 benchlab_multi_device.launch.py

# Configure/activate all devices
ros2 lifecycle set /benchlab_serial_node_0 configure
ros2 lifecycle set /benchlab_serial_node_0 activate
ros2 lifecycle set /benchlab_serial_node_1 configure
ros2 lifecycle set /benchlab_serial_node_1 activate

# Topics will be namespaced:
ros2 topic echo /device0/benchlab/telemetry
ros2 topic echo /device1/benchlab/telemetry

# Services will also be namespaced:
ros2 service call /device0/benchlab/set_name benchlab_msgs/srv/SetDeviceName "name: 'Device0'"
ros2 service call /device1/benchlab/set_name benchlab_msgs/srv/SetDeviceName "name: 'Device1'"
```

## Diagnostics

Monitor node health via `/diagnostics` topic:

```bash
# View diagnostics
ros2 topic echo /diagnostics

# Use rqt_robot_monitor for GUI
rqt_robot_monitor
```

Diagnostics include:
- Connection status
- Read/error counts
- Last error message
- Device identification
- Firmware version

## Lifecycle Management

Both nodes are lifecycle nodes with states:

1. **Unconfigured** → `configure` → **Inactive**
2. **Inactive** → `activate` → **Active** (streaming)
3. **Active** → `deactivate` → **Inactive**
4. **Inactive** → `cleanup` → **Unconfigured**
5. Any state → `shutdown` → **Finalized**

```bash
# Check current state
ros2 lifecycle get /benchlab_serial_node

# Transition through states
ros2 lifecycle set /benchlab_serial_node configure
ros2 lifecycle set /benchlab_serial_node activate

# Stop streaming
ros2 lifecycle set /benchlab_serial_node deactivate

# Resume streaming
ros2 lifecycle set /benchlab_serial_node activate

# Shutdown
ros2 lifecycle set /benchlab_serial_node shutdown
```

## QoS Policies

| Data Type | Reliability | Durability | History | Depth |
|-----------|-------------|------------|---------|-------|
| Telemetry | BEST_EFFORT | VOLATILE | KEEP_LAST | 10 |
| Device Info | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |
| Diagnostics | RELIABLE | VOLATILE | KEEP_LAST | 10 |

**Telemetry** uses BEST_EFFORT for lowest latency (<10ms in serial mode).
**Device Info** uses TRANSIENT_LOCAL so late-joining nodes receive last value.
**Diagnostics** uses RELIABLE for guaranteed delivery.

## Performance

### Direct Serial Mode

- **Latency**: <10ms (native USB access)
- **Throughput**: 10-100 Hz configurable
- **CPU Usage**: ~2-3% at 10 Hz
- **Reliability**: Direct device access, no intermediaries

### HTTP Bridge Mode

- **Latency**: 50-100ms (HTTP overhead + network)
- **Throughput**: 1-10 Hz recommended
- **CPU Usage**: ~5-8% at 10 Hz
- **Reliability**: Depends on benchlabd service availability

**Recommendation**: Use serial mode for robotics applications requiring real-time control and low latency.

## ROS2 Tool Integration

### RViz

Create custom visualization plugins or use `rqt_plot`:

```bash
# Plot chip temperature
rqt_plot /benchlab/telemetry/chip_temp_c

# Plot power consumption
rqt_plot /benchlab/telemetry/power[0]/power_w
```

### PlotJuggler

Structured messages work great with PlotJuggler for real-time data visualization.

### ROS Bags

Record and playback telemetry:

```bash
# Record telemetry
ros2 bag record /benchlab/telemetry /benchlab/device_info /diagnostics

# Playback
ros2 bag play <bag_name>
```

### Foxglove Studio

Structured messages are fully compatible with Foxglove for advanced visualization.

## Troubleshooting

### Node Won't Configure

```bash
# Check device permissions
ls -l /dev/benchlab0

# Ensure you're in dialout group
groups

# Check device is connected
ros2 run benchlab_ros2 benchlab_serial_node --ros-args -p device_path:=/dev/benchlab0
```

### No Telemetry Published

Ensure node is in **Active** state:

```bash
ros2 lifecycle get /benchlab_serial_node
# Should show: active [3]

# If not active:
ros2 lifecycle set /benchlab_serial_node activate
```

### Service Call Fails

Check node diagnostics:

```bash
ros2 topic echo /diagnostics --once
```

### HTTP Mode Connection Issues

Verify benchlabd is running:

```bash
curl http://127.0.0.1:8080/health
```

## Advanced Topics

### Custom QoS Profiles

Modify QoS in your subscriber for specific needs:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,  # Guarantee delivery
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Get last message
    depth=1
)

self.subscription = self.create_subscription(
    BenchLabTelemetry,
    '/benchlab/telemetry',
    self.callback,
    qos
)
```

### Parameter Overrides

Override parameters via command line:

```bash
ros2 run benchlab_ros2 benchlab_serial_node --ros-args \
  -p device_path:=/dev/ttyACM0 \
  -p publish_rate:=20.0 \
  -p device_namespace:=robot1
```

### Integration with Nav2/MoveIt

Telemetry can be used for:
- Temperature monitoring (shutdown on overheat)
- Power consumption tracking
- Fan control based on compute load
- System health diagnostics

Example: Emergency stop on high temperature:

```python
class ThermalMonitor(Node):
    def __init__(self):
        super().__init__('thermal_monitor')
        self.subscription = self.create_subscription(
            BenchLabTelemetry,
            '/benchlab/telemetry',
            self.check_temperature,
            10
        )
        self.emergency_stop_pub = self.create_publisher(EmergencyStop, '/emergency_stop', 10)

    def check_temperature(self, msg):
        if msg.chip_temp_c > 80.0:  # Emergency threshold
            self.get_logger().error(f'OVERHEAT: {msg.chip_temp_c}°C')
            self.emergency_stop_pub.publish(EmergencyStop(stop=True, reason='Thermal'))
```

## Contributing

See main repository for contribution guidelines.

## License

GPL-3.0 (same as main project)

## Support

- **Issues**: https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit/issues
- **Documentation**: https://benchlab.io/docs

---

**Built with Claude Code** for production robotics workflows.
