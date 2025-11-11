# BenchLab Linux Support Kit

[![CI/CD](https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit/actions/workflows/ci.yml/badge.svg)](https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit/actions)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![.NET 8.0](https://img.shields.io/badge/.NET-8.0-512BD4)](https://dotnet.microsoft.com/)

**Linux-native support for BenchLab/OPENBENCHTABLE devices on Ubuntu LTS**. This kit provides device communication, telemetry streaming, and HTTP API access without requiring Windows.

## 🎯 What is This?

LinuxSupportKit enables you to use BenchLab hardware (STM32-based USB CDC devices) on Linux through:
- **Binary protocol implementation** matching BENCHLAB_Core specification
- **Device discovery** via `/dev/serial/by-id` with fallback to `/dev/ttyACM*`
- **REST API** for remote access with authentication
- **Python SDK** for easy integration
- **ROS2 support** for robotics workflows
- **Docker deployment** for containerized environments

## ⚠️ Current Status

This project implements the **complete BenchLab binary protocol** (all 15 commands). Core features are production-ready, with some optimizations in progress:

**Protocol Coverage**: ✅ **15 of 15 commands fully implemented**
- ✅ Device identification and sensor reading
- ✅ Device name read/write
- ✅ Fan profile configuration
- ✅ RGB LED control
- ✅ Calibration management (read/write/load/store)
- ✅ Device UID reading
- ✅ Action commands

**Active Development**:
- Streaming uses polling (100ms interval) - event-driven I/O in progress
- Metrics endpoint is basic (uptime only) - comprehensive metrics in development
- ROS2 integration is minimal - structured messages planned
- Test coverage ~20% - targeting >60%

**Production Readiness**: Fully functional for device control and telemetry monitoring. Performance optimizations and enhanced observability in progress.

## 📦 Components

| Component | Description | Location |
|-----------|-------------|----------|
| **BenchLab.Platform** | Core library (device discovery, binary protocol, serial I/O) | `src/BenchLab.Platform/` |
| **BenchLab.Cli** | Command-line tool (`benchlab-cli`) | `src/BenchLab.Cli/` |
| **BenchLab.Service** | HTTP REST API service (`benchlabd`) | `src/BenchLab.Service/` |
| **Python SDK** | Official Python client library | `python/benchlab_sdk/` |
| **ROS2 Node** | ROS2 telemetry publisher | `python/ros2/` |
| **udev rules** | Device permissions & ModemManager prevention | `udev/99-benchlab.rules` |
| **systemd service** | Daemon configuration | `deploy/systemd/` |
| **Docker** | Container deployment | `Dockerfile`, `docker-compose.yml` |

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 or 24.04 LTS
- .NET 8.0 SDK
- BenchLab/OPENBENCHTABLE device (STM32 VCP, USB VID:PID 0483:5740)

### Installation

```bash
# Install .NET 8
sudo apt update
sudo apt install -y dotnet-sdk-8.0

# Clone repository
git clone https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit.git
cd BENCHLAB.LinuxSupportKit

# Setup permissions
sudo cp udev/99-benchlab.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
sudo usermod -a -G dialout $USER
newgrp dialout  # Or log out/in to apply group membership

# Build
dotnet build -c Release

# Run CLI
cd src/BenchLab.Cli
dotnet run -- list
```

### Quick Test

```bash
# List devices
dotnet run --project src/BenchLab.Cli -- list

# Get device info
dotnet run --project src/BenchLab.Cli -- info --device /dev/benchlab0

# Read sensors (one-shot)
dotnet run --project src/BenchLab.Cli -- sensors --device /dev/benchlab0

# Stream telemetry
dotnet run --project src/BenchLab.Cli -- stream --device /dev/benchlab0
```

## 🔧 CLI Usage

### Commands

```bash
benchlab-cli list [--timeout ms]
  # List all available BenchLab devices

benchlab-cli info --device PATH [--timeout ms]
  # Show device information (name, firmware version, vendor data)

benchlab-cli sensors --device PATH [--timeout ms]
  # Read sensor telemetry (one-shot JSON output)

benchlab-cli stream [--device PATH] [--timeout ms]
  # Stream telemetry data continuously (NDJSON format)

benchlab-cli write --device PATH --text "DATA"
  # Send ASCII data to device
```

### Examples

```bash
# Auto-discover and stream from first BenchLab device
benchlab-cli stream

# Stream from specific device with 1s timeout
benchlab-cli stream --device /dev/benchlab0 --timeout 1000

# Get detailed sensor readings
benchlab-cli sensors --device /dev/benchlab0 | jq '.power[] | select(.power > 50)'
```

## 🌐 HTTP Service

### Running the Service

```bash
# Development (localhost only, no auth)
dotnet run --project src/BenchLab.Service

# Production (with API key)
export BENCHLAB_API_KEY="your-secret-key-here"
export BENCHLAB_BIND_ADDRESS="http://127.0.0.1:8080"
dotnet run --project src/BenchLab.Service
```

### Configuration (Environment Variables)

| Variable | Default | Description |
|----------|---------|-------------|
| `BENCHLAB_BIND_ADDRESS` | `http://127.0.0.1:8080` | Bind address (use `0.0.0.0` for all interfaces) |
| `BENCHLAB_API_KEY` | _(none)_ | API key for Bearer authentication |
| `BENCHLAB_DISCOVERY_TIMEOUT_MS` | `600` | Device discovery timeout |
| `BENCHLAB_COMMAND_TIMEOUT_MS` | `500` | Serial command timeout |

### API Endpoints

| Endpoint | Method | Description | Auth Required |
|----------|--------|-------------|---------------|
| `GET /` | GET | Service information | No |
| `GET /health` | GET | Health check | No |
| `GET /metrics` | GET | Prometheus metrics | No |
| `GET /swagger` | GET | API documentation UI | No |
| `GET /devices` | GET | List available devices | Yes |
| `GET /devices/{id}/info` | GET | Device information | Yes |
| `GET /devices/{id}/sensors` | GET | Sensor telemetry (one-shot) | Yes |
| `GET /stream?device=...` | GET | Stream telemetry (NDJSON) | Yes |
| `POST /write` | POST | Write data to device | Yes |

### API Examples

```bash
# Without authentication (dev mode)
curl http://localhost:8080/devices
curl http://localhost:8080/devices/benchlab0/sensors

# With API key
export API_KEY="your-secret-key"
curl -H "Authorization: Bearer $API_KEY" http://localhost:8080/devices

# Stream telemetry
curl -H "Authorization: Bearer $API_KEY" http://localhost:8080/stream?device=benchlab0

# Write data
curl -X POST -H "Authorization: Bearer $API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"device":"/dev/benchlab0","data":"command"}' \
  http://localhost:8080/write
```

## 🐍 Python SDK

### Installation

```bash
cd python
pip install -e .  # Development install

# Or from PyPI (when published)
pip install benchlab-sdk
```

### Usage

```python
from benchlab_sdk import BenchLabClient

# Create client with authentication
client = BenchLabClient(
    base_url="http://localhost:8080",
    api_key="your-secret-key"
)

# List devices
devices = client.list_devices()
print(f"Found {len(devices)} devices")

# Get device info
info = client.get_device_info("/dev/benchlab0")
print(f"Device: {info.name} (FW v{info.firmware_version})")

# Read sensors (one-shot)
sensors = client.read_sensors("/dev/benchlab0")
print(f"Chip temp: {sensors.temperatures['chip']}°C")
print(f"Total power: {sum(p['power'] for p in sensors.power):.2f}W")

# Stream telemetry
for reading in client.stream_telemetry("/dev/benchlab0"):
    temp = reading['chipTemp']
    power = sum(p['w'] for p in reading['power'])
    print(f"Temp: {temp}°C, Power: {power:.2f}W")
```

### Context Manager Support

```python
with BenchLabClient("http://localhost:8080", api_key="key") as client:
    sensors = client.read_sensors("/dev/benchlab0")
    # Client automatically closed
```

## 🐳 Docker Deployment

### Using Docker Compose (Recommended)

```bash
# Set API key
export BENCHLAB_API_KEY="your-production-key"

# Start service
docker-compose up -d

# View logs
docker-compose logs -f benchlabd

# Stop service
docker-compose down
```

### Manual Docker

```bash
# Build image
docker build -t benchlab/service:latest .

# Run with device passthrough
docker run -d \
  --name benchlabd \
  --device=/dev/ttyACM0 \
  -p 8080:8080 \
  -e BENCHLAB_API_KEY="your-key" \
  benchlab/service:latest
```

### Kubernetes Deployment

See `deploy/kubernetes/` for Helm charts and manifests (TODO).

## 🤖 ROS2 Integration

```bash
cd python/ros2
pip install -r requirements.txt

# Run ROS2 node
python3 benchlab_ros2_node.py
```

Publishes telemetry to `/benchlab/telemetry` topic.

## 🔐 Security Best Practices

### Production Deployment Checklist

- ✅ **Enable authentication**: Set `BENCHLAB_API_KEY` environment variable
- ✅ **Bind to localhost**: Use `BENCHLAB_BIND_ADDRESS=http://127.0.0.1:8080`
- ✅ **Use reverse proxy**: Deploy nginx/caddy with TLS in front of service
- ✅ **Firewall rules**: Restrict access to port 8080
- ✅ **Run as non-root**: Service runs as `benchlab` user in systemd/Docker
- ✅ **Update regularly**: Keep .NET runtime and dependencies patched

### Example nginx Reverse Proxy

```nginx
server {
    listen 443 ssl http2;
    server_name benchlab.example.com;

    ssl_certificate /etc/letsencrypt/live/benchlab.example.com/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/benchlab.example.com/privkey.pem;

    location / {
        proxy_pass http://127.0.0.1:8080;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }
}
```

## 📊 Monitoring & Observability

### Health Check

```bash
curl http://localhost:8080/health
# {"status":"healthy","timestamp":"2025-01-11T...","uptime":123.45}
```

### Prometheus Metrics

```bash
curl http://localhost:8080/metrics
# HELP benchlab_uptime_seconds Service uptime in seconds
# TYPE benchlab_uptime_seconds counter
# benchlab_uptime_seconds 1234.56
```

### Grafana Dashboard

Import `deploy/grafana/dashboard.json` for pre-built monitoring dashboard (TODO).

## 🧪 Testing

### Run Unit Tests

```bash
dotnet test --collect:"XPlat Code Coverage"
```

### Test Coverage

```bash
dotnet test --collect:"XPlat Code Coverage"
reportgenerator -reports:"**/coverage.cobertura.xml" -targetdir:"coverage" -reporttypes:Html
```

**Current Status**: Basic unit tests for protocol and structs (~20% coverage)
**Goal**: >60% code coverage with comprehensive integration tests

## 📚 Protocol Specification

### BenchLab Binary Protocol

The service implements the **complete** official BenchLab binary protocol:

- **Baud Rate**: 115200, 8N1, DTR/RTS enabled
- **Framing**: Fixed-length binary responses per command
- **Byte Order**: Little-endian (STM32 native)
- **Protocol Version**: **Complete implementation - all 15 commands functional** ✅

#### Complete Command Reference

| Command | ID | Description | Response Size |
|---------|-----|-------------|---------------|
| `UART_CMD_WELCOME` | 0x00 | Device identification | 13 bytes ("BENCHLAB\x00") |
| `UART_CMD_READ_SENSORS` | 0x01 | Read all sensors | ~194 bytes (SensorStruct) |
| `UART_CMD_ACTION` | 0x02 | Execute device action | 1 byte (status) |
| `UART_CMD_READ_NAME` | 0x03 | Get device name | 32 bytes |
| `UART_CMD_WRITE_NAME` | 0x04 | Set device name | 1 byte (status) |
| `UART_CMD_READ_FAN_PROFILE` | 0x05 | Get fan settings | 8 bytes (FanProfileStruct) |
| `UART_CMD_WRITE_FAN_PROFILE` | 0x06 | Set fan settings | 1 byte (status) |
| `UART_CMD_READ_RGB` | 0x07 | Get RGB LED settings | 8 bytes (RgbStruct) |
| `UART_CMD_WRITE_RGB` | 0x08 | Set RGB LEDs | 1 byte (status) |
| `UART_CMD_READ_CALIBRATION` | 0x09 | Get calibration data | Variable (CalibrationStruct) |
| `UART_CMD_WRITE_CALIBRATION` | 0x0A | Apply calibration | 1 byte (status) |
| `UART_CMD_LOAD_CALIBRATION` | 0x0B | Load from flash | 1 byte (status) |
| `UART_CMD_STORE_CALIBRATION` | 0x0C | Save to flash | 1 byte (status) |
| `UART_CMD_READ_UID` | 0x0D | Read device UID | 12 bytes (96-bit UID) |
| `UART_CMD_READ_VENDOR_DATA` | 0x0E | Get vendor info | 3 bytes (VID, PID, FW version) |

**All commands are fully implemented** in `BinaryProtocol.cs` with proper struct marshalling and error handling.

#### Data Structures

**VendorDataStruct** (3 bytes):
- `VendorId`: 0xEE (BenchLab)
- `ProductId`: 0x10 (standard device)
- `FwVersion`: Firmware version number

**SensorStruct** (~194 bytes):
- 13 voltage inputs (millivolts)
- 11 power channels (voltage, current, power)
- 9 fan channels (enable, duty, RPM)
- Temperature sensors (chip, ambient, 4x additional)
- Humidity sensor

See `src/BenchLab.Platform/Protocol/` for complete implementation.

## 🛠️ Development

### Build from Source

```bash
git clone https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit.git
cd BENCHLAB.LinuxSupportKit

# Restore dependencies
dotnet restore

# Build all projects
dotnet build

# Run tests
dotnet test

# Publish single-file binaries
dotnet publish src/BenchLab.Cli -c Release -r linux-x64 --self-contained -p:PublishSingleFile=true
dotnet publish src/BenchLab.Service -c Release -r linux-x64 --self-contained -p:PublishSingleFile=true
```

### Project Structure

```
BENCHLAB.LinuxSupportKit/
├── src/
│   ├── BenchLab.Platform/       # Core library
│   │   ├── Discovery/           # Device enumeration
│   │   ├── Ports/               # Serial port abstraction
│   │   └── Protocol/            # Binary protocol implementation
│   ├── BenchLab.Cli/            # Command-line tool
│   └── BenchLab.Service/        # HTTP REST API
├── tests/
│   └── BenchLab.Platform.Tests/ # Unit tests
├── python/
│   ├── benchlab_sdk/            # Python SDK
│   ├── clients/                 # Example clients
│   └── ros2/                    # ROS2 integration
├── udev/                        # udev rules
├── deploy/                      # Deployment configs
│   ├── systemd/
│   └── kubernetes/
├── Dockerfile
└── docker-compose.yml
```

## 🐛 Troubleshooting

### Device Not Detected

```bash
# Check if device is connected
lsusb | grep 0483:5740

# Check permissions
ls -l /dev/ttyACM*

# Reload udev rules
sudo udevadm control --reload && sudo udevadm trigger

# Check group membership
groups  # Should include 'dialout'
```

### ModemManager Interference

If ModemManager is hijacking your device:

```bash
# Verify udev rule is active
udevadm info /dev/ttyACM0 | grep ID_MM_DEVICE_IGNORE

# Should output: E: ID_MM_DEVICE_IGNORE=1
```

### Service Won't Start

```bash
# Check logs
journalctl -u benchlab -f

# Test manually
cd /opt/benchlab
./benchlabd

# Check port binding
sudo netstat -tlnp | grep 8080
```

### API Returns 401 Unauthorized

- Ensure `BENCHLAB_API_KEY` is set in environment
- Include `Authorization: Bearer <key>` header in requests
- Verify key matches between service and client

## 🤝 Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Push to branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style

- Follow C# conventions (PascalCase for public members)
- Use nullable reference types
- Add XML doc comments for public APIs
- Write unit tests for new features
- Run `dotnet format` before committing

## 📄 License

GNU General Public License v3.0 (GPL-3.0)

This license allows easy integration into GPL-3.0 projects like OPENBENCHTABLE/BENCHLAB_Core.

## 🔗 Related Projects

- [BENCHLAB.BENCHLAB_Core](https://github.com/BenchLab-io/BENCHLAB.BENCHLAB_Core) - Core BenchLab software
- [BENCHLAB.PythonSample](https://github.com/BenchLab-io/BENCHLAB.PythonSample) - Python examples
- [OPENBENCHTABLE](https://benchlab.io/) - Open-source bench testing platform

## 📞 Support

- **Documentation**: https://benchlab.io/docs
- **Issues**: https://github.com/BenchLab-io/BENCHLAB.LinuxSupportKit/issues
- **Website**: https://benchlab.io

## ✨ Acknowledgments

Built with ❤️ for the BenchLab and OPENBENCHTABLE community.

Special thanks to all contributors and the open-source community!

---

**Made with Claude Code** - AI-assisted development for production-quality software.
