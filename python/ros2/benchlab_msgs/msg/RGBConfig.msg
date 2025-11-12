# RGB LED configuration
# Maps to RgbStruct (8 bytes)

uint8 MODE_OFF=0
uint8 MODE_SOLID=1
uint8 MODE_BREATHING=2
uint8 MODE_CYCLE=3
uint8 MODE_TEMPERATURE=4

uint8 mode           # RGB mode: 0=Off, 1=Solid, 2=Breathing, 3=Cycle, 4=Temperature
uint8 red            # Red channel (0-255)
uint8 green          # Green channel (0-255)
uint8 blue           # Blue channel (0-255)
uint8 brightness     # Brightness (0-255)
uint8 speed          # Animation speed for modes 2-4 (0-255, higher=faster)
