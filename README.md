# BenchLab Linux Support Kit (Ubuntu LTS)

This kit provides **Linux-first** components to use BenchLab devices on Ubuntu LTS without requiring Windows.
It is designed to be integrated upstream into `BENCHLAB.BENCHLAB_Core` or used standalone.

## Components

- **BenchLab.Platform** (`src/BenchLab.Platform`): .NET 8 library for device discovery and serial I/O on Linux.
- **BenchLab.Cli** (`src/BenchLab.Cli`): console tool to list devices and stream telemetry (NDJSON or CSV).
- **BenchLab.Service** (`src/BenchLab.Service`): HTTP service that exposes `/devices` and `/stream` endpoints.
- **udev** (`udev/99-benchlab.rules`): permissions + ModemManager ignore.
- **systemd** (`deploy/systemd/benchlab.service`): run the service as a daemon.
- **CI** (`.github/workflows/ci.yml`): Linux build + publish.
- **Python client & ROS2** (`python/clients`, `python/ros2`): reference consumers.

> NOTE: The kit does not depend on the proprietary Windows app. It focuses on Linux serial, enumeration and streaming.
> To integrate with `BENCHLAB.BENCHLAB_Core`, use `BenchLab.Platform`'s `PortDiscovery` + `SerialPortAdapter`.

## Quickstart (Ubuntu 22.04/24.04 LTS)

```bash
sudo apt update
sudo apt install -y dotnet-sdk-8.0
# permissions for serial
sudo usermod -a -G dialout $USER
sudo cp udev/99-benchlab.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
newgrp dialout

# build CLI and Service
cd src/BenchLab.Cli && dotnet publish -c Release -r linux-x64 --self-contained true -p:PublishSingleFile=true
cd ../BenchLab.Service && dotnet publish -c Release -r linux-x64 --self-contained true -p:PublishSingleFile=true

# list devices (by-id preferred; falls back to ttyACM*); requires BenchLab device attached
./bin/Release/net8.0/linux-x64/publish/benchlab-cli list

# stream telemetry (raw lines or hex NDJSON) from the first detected device
./bin/Release/net8.0/linux-x64/publish/benchlab-cli stream

# run service (default http://0.0.0.0:8080)
./bin/Release/net8.0/linux-x64/publish/benchlabd
```

## Systemd daemon (optional)

```bash
sudo mkdir -p /opt/benchlab
sudo cp -r src/BenchLab.Service/bin/Release/net8.0/linux-x64/publish/* /opt/benchlab/
sudo cp deploy/systemd/benchlab.service /etc/systemd/system/benchlab.service
sudo systemctl daemon-reload
sudo systemctl enable --now benchlab.service
```

## API

- `GET /devices` → JSON list of candidate devices with path and probe details.
- `GET /stream?device=/dev/benchlab0` → NDJSON stream of raw serial (UTF-8 lines if available; else hex frames).
- `POST /write` with body `{ "device": "...", "data": "ASCII string" }` → writes to serial (for simple tests).

## ROS2 and Python

- `python/clients/benchlab_client.py` streams NDJSON and prints JSON.
- `python/ros2/benchlab_ros2_node.py` publishes a sample topic from the stream.

## Packaging

- `dotnet publish` creates single-file binaries.
- Optional Debian packaging scaffolding is provided in `packaging/deb/`.

## License

GPL-3.0 (to ease upstreaming to GPL-3.0 projects). See `LICENSE`.
