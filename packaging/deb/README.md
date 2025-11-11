# Debian Package Structure

This directory contains the Debian package structure for BenchLab Linux Support Kit.

## Directory Structure

```
packaging/deb/
├── DEBIAN/
│   ├── control         # Package metadata
│   ├── postinst        # Post-installation script
│   ├── prerm           # Pre-removal script
│   └── postrm          # Post-removal script
├── usr/
│   ├── bin/            # Binaries (benchlab-cli, benchlabd)
│   └── lib/systemd/system/  # Systemd unit files
└── etc/
    └── udev/rules.d/   # udev rules for device detection
```

## Building the Package

### Prerequisites

```bash
sudo apt-get install dpkg-dev
```

### Build Steps

1. **Build the binaries:**

```bash
cd src/BenchLab.Cli
dotnet publish -c Release -r linux-x64 --self-contained false -o ../../packaging/deb/usr/bin/

cd ../BenchLab.Service
dotnet publish -c Release -r linux-x64 --self-contained false -o ../../packaging/deb/usr/bin/
```

2. **Copy systemd unit and udev rules:**

```bash
cp systemd/benchlabd.service packaging/deb/usr/lib/systemd/system/
cp udev/99-benchlab.rules packaging/deb/etc/udev/rules.d/
```

3. **Set proper permissions:**

```bash
chmod 755 packaging/deb/DEBIAN/postinst
chmod 755 packaging/deb/DEBIAN/prerm
chmod 755 packaging/deb/DEBIAN/postrm
```

4. **Build the .deb package:**

```bash
dpkg-deb --build packaging/deb benchlabd_0.1.0_amd64.deb
```

## Installing the Package

```bash
sudo dpkg -i benchlabd_0.1.0_amd64.deb
sudo apt-get install -f  # Install dependencies if needed
```

## Verifying Installation

```bash
# Check service status
systemctl status benchlabd

# Check CLI availability
benchlab-cli --help

# View logs
journalctl -u benchlabd -f
```

## Uninstalling

```bash
# Remove package (keep config)
sudo apt-get remove benchlabd

# Purge package (remove everything)
sudo apt-get purge benchlabd
```

## Package Details

- **Package Name:** benchlabd
- **Version:** 0.1.0
- **Architecture:** amd64
- **Depends:** systemd

## What Gets Installed

- `/usr/bin/benchlab-cli` - Command-line interface
- `/usr/bin/benchlabd` - HTTP service daemon
- `/usr/lib/systemd/system/benchlabd.service` - Systemd unit file
- `/etc/udev/rules.d/99-benchlab.rules` - udev rules for device detection

## Post-Installation Actions

The package automatically:
1. Creates `benchlab` system user and group
2. Adds `benchlab` user to `dialout` group (for serial port access)
3. Reloads udev rules
4. Enables and starts `benchlabd.service`

## Troubleshooting

### Service Won't Start

```bash
# Check logs
journalctl -u benchlabd --no-pager

# Check permissions
ls -la /dev/ttyACM* /dev/ttyUSB*
groups benchlab
```

### Device Not Detected

```bash
# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Check device presence
benchlab-cli list
```

### Permission Denied

```bash
# Ensure user is in dialout group
sudo usermod -a -G dialout $USER

# Re-login required for group changes to take effect
```
