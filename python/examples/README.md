# BenchLab Python SDK Examples

This directory contains example scripts demonstrating how to use the BenchLab Python SDK.

## Prerequisites

1. **BenchLab HTTP Service** must be running:
   ```bash
   # Start the service
   sudo systemctl start benchlab-http

   # Check status
   sudo systemctl status benchlab-http

   # Or run manually (if not installed as service)
   cd ~/BenchLab/LinuxSupportKit
   dotnet run --project src/BenchLab.Service
   ```

2. **Python Dependencies**:
   ```bash
   pip install requests
   ```

3. **Device Access**: Ensure your user has permission to access BenchLab devices:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in for group changes to take effect
   ```

## Examples

### 1. Device Information (`basic_info.py`)

Query device information and calibration status.

```bash
python3 basic_info.py
```

**What it demonstrates:**
- Service health check
- Device discovery
- Querying device details (name, firmware, vendor/product IDs)
- Reading calibration data

### 2. Sensor Streaming (`stream_sensors.py`)

Stream real-time telemetry data from a device.

```bash
# Auto-discover device
python3 stream_sensors.py

# Specific device
python3 stream_sensors.py /dev/benchlab0
python3 stream_sensors.py benchlab0
```

**What it demonstrates:**
- Continuous telemetry streaming (NDJSON)
- Parsing sensor readings
- Displaying power consumption and temperatures
- Fan speed monitoring

Press `Ctrl+C` to stop streaming.

### 3. Fan Control (`fan_control.py`)

Control fan speed with manual and automatic modes.

```bash
# Control fan 0 on first device
python3 fan_control.py /dev/benchlab0

# Control fan 1
python3 fan_control.py benchlab0 1
```

**What it demonstrates:**
- Setting fan to manual mode (specific PWM duty)
- Setting fan to automatic temperature-based control
- Adjusting fan speed (0-100%)
- Temperature threshold configuration

### 4. RGB LED Control (`rgb_control.py`)

Control RGB LED colors and animation modes.

```bash
python3 rgb_control.py /dev/benchlab0
```

**What it demonstrates:**
- Setting solid colors (red, green, blue)
- Breathing animation mode
- Color cycle mode
- Temperature-based color mode
- Turning LED off
- Brightness and speed control

## SDK Usage Patterns

### Basic Connection

```python
from benchlab_client import BenchLabClient

# Default connection (http://127.0.0.1:8080)
client = BenchLabClient()

# Custom connection
client = BenchLabClient(
    base_url="http://192.168.1.100:8080",
    api_key="your-secret-key",
    timeout=10.0
)
```

### Error Handling

```python
try:
    info = client.get_device_info("/dev/benchlab0")
    print(f"Device: {info.name}")
except DeviceNotFoundError:
    print("Device not found")
except AuthenticationError:
    print("Invalid API key")
except BenchLabError as e:
    print(f"Error: {e}")
```

### Context Manager

```python
with BenchLabClient() as client:
    devices = client.list_devices()
    for dev in devices:
        print(dev['device'])
# Client automatically closed
```

### Streaming with Reconnection

```python
try:
    for reading in client.stream_telemetry(device, max_retries=5):
        # Process reading
        temp = reading['chipTemp']
        print(f"Temperature: {temp}°C")
except BenchLabError as e:
    print(f"Stream failed: {e}")
```

## Troubleshooting

### Service Not Running

```
✗ Service not available: Connection failed
```

**Solution:** Start the BenchLab HTTP service:
```bash
sudo systemctl start benchlab-http
```

### Device Not Found

```
✗ Device not found
```

**Solution:** Check device is connected and permissions are correct:
```bash
# List USB devices
lsusb | grep BenchLab

# Check device permissions
ls -l /dev/benchlab*
```

### Permission Denied

```
✗ Access denied to device
```

**Solution:** Add your user to the `dialout` group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Import Error

```
ModuleNotFoundError: No module named 'benchlab_client'
```

**Solution:** The examples add the SDK to Python path automatically. If this fails, install the SDK:
```bash
cd python/benchlab_sdk
pip install -e .
```

## Next Steps

- Explore the [main README](../../README.md) for full documentation
- Check [ROS2 integration](../ros2/) for robotics workflows
- View [API reference](../benchlab_sdk/benchlab_client.py) for all available methods

## Support

- **Issues**: https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit/issues
- **Documentation**: https://benchlab.io/docs
