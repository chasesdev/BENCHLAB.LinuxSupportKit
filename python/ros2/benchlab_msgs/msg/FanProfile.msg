# Fan profile configuration
# Maps to FanProfileStruct (8 bytes)
# Controls how a fan responds to temperature or manual control

uint8 MODE_MANUAL=0
uint8 MODE_AUTO=1

uint8 mode               # 0=Manual, 1=Auto (temperature-based)
uint8 manual_duty        # Manual duty cycle (0-255) when mode=0
float32 temp_threshold_c # Temperature threshold for auto mode (degrees C)
uint8 min_duty           # Minimum duty cycle in auto mode (0-255)
uint8 max_duty           # Maximum duty cycle in auto mode (0-255)
uint8 sensor_index       # Temperature sensor index to use for auto mode
