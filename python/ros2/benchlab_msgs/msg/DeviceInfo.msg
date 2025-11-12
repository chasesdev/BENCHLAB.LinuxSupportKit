# BenchLab device information
# Combines data from VendorDataStruct and device discovery

std_msgs/Header header

string device_path      # Device path (e.g., /dev/benchlab0)
string device_name      # User-assigned device name (max 32 chars)
string uid              # 96-bit unique identifier (hex string)
uint8 vendor_id         # Vendor ID (0xEE for BenchLab)
uint8 product_id        # Product ID (0x10 for standard device)
uint8 firmware_version  # Firmware version number
