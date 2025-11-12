# Complete BenchLab sensor telemetry
# Maps to SensorStruct (~194 bytes)

std_msgs/Header header

# Voltage channels (13 channels in millivolts)
int16[13] voltages_mv

# Digital supply and reference voltages
uint16 vdd_mv         # Digital supply voltage in millivolts
uint16 vref_mv        # Reference voltage in millivolts

# Temperature sensors (all in degrees Celsius)
float64 chip_temp_c   # Chip temperature
float64 temp1_c       # Temperature sensor 1
float64 temp2_c       # Temperature sensor 2
float64 temp3_c       # Temperature sensor 3
float64 temp4_c       # Temperature sensor 4
float64 ambient_temp_c # Ambient temperature

# Humidity
float64 humidity_percent

# Status bits
uint8 fan_switch_status
uint8 rgb_switch_status
uint8 rgb_ext_status
uint8 fan_ext_duty

# Power measurements (11 channels)
PowerSensor[11] power

# Fan measurements (9 channels)
FanSensor[9] fans
