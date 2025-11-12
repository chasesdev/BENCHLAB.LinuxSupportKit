# Calibration data structure
# Maps to CalibrationStruct
# Stores offset and scaling factors for sensors

# Voltage calibration (13 channels)
int16[13] voltage_offsets_mv  # Voltage calibration offsets in millivolts
int16[13] voltage_scales      # Voltage calibration scales (factor × 1000)

# Temperature calibration
int16 temp_offset             # Temperature calibration offset (degrees C × 100)
int16 temp_scale              # Temperature calibration scale (factor × 1000)

# Current calibration (11 channels)
int32[11] current_offsets_ma  # Current calibration offsets in milliamps
int16[11] current_scales      # Current calibration scales (factor × 1000)

# Calibration flags (bit field for which sensors are calibrated)
uint32 flags
