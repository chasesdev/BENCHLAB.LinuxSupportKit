# Fan sensor and control data
# Maps to FanSensor struct (4 bytes)

bool enabled           # Fan enable status
uint8 duty_cycle       # PWM duty cycle (0-255)
uint16 rpm             # Tachometer reading in RPM
float32 duty_percent   # Duty cycle as percentage (0-100)
